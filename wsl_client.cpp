// ============================================================================
// wsl_client.cpp  —  C++23  Webots 仿真传感器数据接收客户端 (for WSL)
//
// 编译:
//   g++-16 -std=c++23 -O2 -o wsl_client wsl_client.cpp
//
// 运行:
//   ./wsl_client                     (默认 127.0.0.1:12345)
//   ./wsl_client --host 172.x.x.x   (指定 Windows 主机 IP)
//
// 协议:
//   与 my_controller.py 中的二进制协议完全一致:
//   [type:1B][timestamp_us:8B][payload_len:4B][payload:payload_len B]
//
//   消息类型:
//     0x01  —  相机帧  (Camera Frame)
//     0x02  —  LiDAR 点云 (Lidar PointCloud)
//     0xFF  —  元信息  (Metadata JSON)
// ============================================================================

#include <print>                     // C++23: std::println
#include <cstdint>
#include <vector>
#include <string>
#include <string_view>
#include <cmath>
#include <span>
#include <optional>
#include <chrono>
#include <thread>
#include <expected>
#include <array>
#include <numbers>

// ── POSIX socket (Linux/WSL 标准) ──────────────────────────────────────────
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <netdb.h>
#include <cstring>

// ============================================================================
//  协议常量 & 工具
// ============================================================================

namespace proto {

    // 消息类型枚举
    enum Type : uint8_t {
        CAMERA = 0x01,
        LIDAR  = 0x02,
        INFO   = 0xFF,
    };

    // 头部固定 13 字节
    inline constexpr size_t HEADER_SIZE = 13;

    // 线缆格式 (所有多字节为大端序):
    //   [0]    type          uint8
    //   [1-8]  timestamp_us  uint64
    //   [9-12] payload_len   uint32
    //   [13+]  payload       uint8[]

    // 从大端字节流中读取整数的辅助函数 (纯手工, 无依赖)
    [[nodiscard]] constexpr uint16_t read_be16(const uint8_t* p) noexcept {
        return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
    }
    [[nodiscard]] constexpr uint32_t read_be32(const uint8_t* p) noexcept {
        return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16)
             | (uint32_t(p[2]) <<  8) | uint32_t(p[3]);
    }
    [[nodiscard]] constexpr uint64_t read_be64(const uint8_t* p) noexcept {
        return (uint64_t(p[0]) << 56) | (uint64_t(p[1]) << 48)
             | (uint64_t(p[2]) << 40) | (uint64_t(p[3]) << 32)
             | (uint64_t(p[4]) << 24) | (uint64_t(p[5]) << 16)
             | (uint64_t(p[6]) <<  8) | uint64_t(p[7]);
    }

    // 消息头结构 (解析后)
    struct Header {
        Type     type;
        uint64_t timestamp_us;
        uint32_t payload_len;
    };

    // 从 13 字节裸数据解析头部
    [[nodiscard]] Header parse_header(std::span<const uint8_t, HEADER_SIZE> raw) noexcept {
        return Header{
            .type         = static_cast<Type>(raw[0]),
            .timestamp_us = read_be64(raw.data() + 1),
            .payload_len  = read_be32(raw.data() + 9),
        };
    }

    // 浮点数从大端 bytes → float (网络传输的 float32 是 IEEE 754, 字节序问题而已)
    // 注意: x86 是小端, 线缆上是 big-endian, 所以需要字节反转
    [[nodiscard]] float read_be_float(const uint8_t* p) noexcept {
        uint32_t bits = read_be32(p);
        float result;
        std::memcpy(&result, &bits, sizeof(result));
        return result;
    }

} // namespace proto


// ============================================================================
//  传感器数据结构
// ============================================================================

// 相机帧
struct CameraFrame {
    uint16_t            width;
    uint16_t            height;
    std::vector<uint8_t> image;  // BGRA, size = width * height * 4
};

// LiDAR 帧 (原始距离)
struct LidarFrame {
    uint16_t         num_layers;
    uint16_t         horiz_res;
    std::vector<float> ranges;   // size = num_layers * horiz_res
};

// 3D 点
struct Point3D {
    double x, y, z;
};

// 元信息 (从 JSON 中提取的字段)
struct SensorInfo {
    double lidar_fov  = 3.0;    // 水平视场角 [rad], fallback 与 .wbt 一致
    double lidar_vfov = 1.03;   // 垂直视场角 [rad]
    // 可根据需要扩展更多字段
};


// ============================================================================
//  TCP 连接管理 (RAII)
// ============================================================================

class TcpClient {
public:
    TcpClient() = default;
    ~TcpClient() { close(); }

    // 不允许拷贝
    TcpClient(const TcpClient&) = delete;
    TcpClient& operator=(const TcpClient&) = delete;

    // 允许移动
    TcpClient(TcpClient&& other) noexcept
        : fd_(other.fd_) { other.fd_ = -1; }
    TcpClient& operator=(TcpClient&& other) noexcept {
        if (this != &other) {
            close();
            fd_ = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }

    // 连接到 host:port
    void connect(const std::string& host, uint16_t port) {
        if (fd_ >= 0) close();

        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0)
            throw std::runtime_error("socket() failed: " + std::string(std::strerror(errno)));

        // 禁用 Nagle 算法 (低延迟)
        int flag = 1;
        if (::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) < 0)
            throw std::runtime_error("setsockopt(TCP_NODELAY) failed");

        // 解析地址
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);

        if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) <= 0) {
            // 不是点分十进制? 尝试 DNS 解析
            struct hostent* he = ::gethostbyname(host.c_str());
            if (!he)
                throw std::runtime_error("Failed to resolve host: " + host);
            std::memcpy(&addr.sin_addr, he->h_addr, he->h_length);
        }

        // 连接 (带超时: 先设非阻塞, connect, 再设回阻塞)
        ::fcntl(fd_, F_SETFL, O_NONBLOCK);

        int rc = ::connect(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
        if (rc < 0) {
            if (errno == EINPROGRESS) {
                struct timeval tv{.tv_sec = 5, .tv_usec = 0};  // 5s 超时
                fd_set wset;
                FD_ZERO(&wset);
                FD_SET(fd_, &wset);
                rc = ::select(fd_ + 1, nullptr, &wset, nullptr, &tv);
                if (rc <= 0) {
                    close();
                    throw std::runtime_error("Connection timeout to " + host + ":" + std::to_string(port));
                }
            } else {
                close();
                throw std::runtime_error("connect() failed: " + std::string(std::strerror(errno)));
            }
        }

        // 恢复阻塞模式
        ::fcntl(fd_, F_SETFL, 0);
    }

    // 精确读取 n 字节 (TCP 流式协议的关键!)
    void recv_exact(std::span<uint8_t> buf) {
        size_t total = 0;
        while (total < buf.size()) {
            ssize_t n = ::read(fd_, buf.data() + total, buf.size() - total);
            if (n < 0) {
                if (errno == EINTR) continue;       // 被信号中断, 重试
                throw std::runtime_error("read() failed: " + std::string(std::strerror(errno)));
            }
            if (n == 0)                              // 对端关闭连接
                throw std::runtime_error("Connection closed by peer");
            total += static_cast<size_t>(n);
        }
    }

    // 方便的重载: 读取到 vector 中
    void recv_exact(std::vector<uint8_t>& buf) {
        recv_exact(std::span<uint8_t>{buf.data(), buf.size()});
    }

    void close() noexcept {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    [[nodiscard]] int fd() const noexcept { return fd_; }

private:
    int fd_ = -1;
};


// ============================================================================
//  元信息 JSON 解析 (极简版 — 只提取我们需要的字段)
// ============================================================================

// 从 JSON 字符串中查找一个数字类型的字段值
// 例如: "fov": 3.0   → 返回 3.0
[[nodiscard]] static
std::optional<double> json_extract_double(std::string_view json, std::string_view key) {
    auto key_start = json.find(key);
    if (key_start == std::string_view::npos) return std::nullopt;

    auto colon = json.find(':', key_start + key.size());
    if (colon == std::string_view::npos) return std::nullopt;

    // 跳过空白
    auto val_start = json.find_first_not_of(" \t\n\r", colon + 1);
    if (val_start == std::string_view::npos) return std::nullopt;

    // 末尾: 遇到逗号 / 右花括号 / 空白
    auto val_end = val_start;
    while (val_end < json.size() && json[val_end] != ',' && json[val_end] != '}'
           && json[val_end] != ']' && json[val_end] != '\n' && json[val_end] != ' ') {
        ++val_end;
    }

    try {
        std::string num{json.substr(val_start, val_end - val_start)};
        return std::stod(num);
    } catch (...) {
        return std::nullopt;
    }
}

// 解析元信息消息, 提取传感器参数
[[nodiscard]] static
SensorInfo parse_metadata(std::string_view json) {
    SensorInfo info;

    // 在 "lidar" 块中查找
    auto lidar_pos = json.find("\"lidar\"");
    if (lidar_pos == std::string_view::npos) return info;

    // 从 lidar 块开始往后找
    auto lidar_block = json.substr(lidar_pos);

    if (auto fov = json_extract_double(lidar_block, "\"fov\""))
        info.lidar_fov = *fov;
    if (auto vfov = json_extract_double(lidar_block, "\"vertical_fov\""))
        info.lidar_vfov = *vfov;

    std::println(stderr, "[WSL]  Lidar FOV: {:.3f} rad (H) × {:.3f} rad (V)",
                 info.lidar_fov, info.lidar_vfov);
    return info;
}


// ============================================================================
//  消息反序列化
// ============================================================================

[[nodiscard]] CameraFrame parse_camera(std::span<const uint8_t> payload) {
    if (payload.size() < 4)
        throw std::runtime_error("Camera payload too short");

    uint16_t w = proto::read_be16(payload.data());
    uint16_t h = proto::read_be16(payload.data() + 2);

    size_t expected = static_cast<size_t>(w) * h * 4;
    if (payload.size() != expected + 4)
        throw std::runtime_error("Camera payload size mismatch");

    CameraFrame frame{
        .width  = w,
        .height = h,
        .image  = std::vector<uint8_t>(payload.begin() + 4, payload.end()),
    };
    return frame;
}

[[nodiscard]] LidarFrame parse_lidar(std::span<const uint8_t> payload) {
    if (payload.size() < 4)
        throw std::runtime_error("Lidar payload too short");

    uint16_t layers  = proto::read_be16(payload.data());
    uint16_t hres    = proto::read_be16(payload.data() + 2);
    size_t   n_pts   = static_cast<size_t>(layers) * hres;

    size_t expected = 4 + n_pts * 4;  // 4 bytes per float32
    if (payload.size() != expected)
        throw std::runtime_error("Lidar payload size mismatch");

    std::vector<float> ranges(n_pts);
    for (size_t i = 0; i < n_pts; ++i) {
        ranges[i] = proto::read_be_float(payload.data() + 4 + i * 4);
    }

    return LidarFrame{
        .num_layers = layers,
        .horiz_res  = hres,
        .ranges     = std::move(ranges),
    };
}


// ============================================================================
//  LiDAR 距离 → 3D 点云 (球坐标 → 笛卡尔坐标)
// ============================================================================

[[nodiscard]] std::vector<Point3D> lidar_to_pointcloud(
    const LidarFrame& lidar, double fov, double vfov)
{
    // 生成水平 / 垂直角度网格
    std::vector<double> h_angles(lidar.horiz_res);
    std::vector<double> v_angles(lidar.num_layers);

    for (uint16_t i = 0; i < lidar.horiz_res; ++i) {
        h_angles[i] = -fov / 2.0 + (static_cast<double>(i) + 0.5) * fov / lidar.horiz_res;
    }
    for (uint16_t j = 0; j < lidar.num_layers; ++j) {
        v_angles[j] = -vfov / 2.0 + (static_cast<double>(j) + 0.5) * vfov / lidar.num_layers;
    }

    std::vector<Point3D> points;
    // 预分配 (大部分点都有效, 除了 out-of-range 的)
    points.reserve(lidar.ranges.size());

    for (uint16_t j = 0; j < lidar.num_layers; ++j) {
        double phi = v_angles[j];              // 垂直角 (俯仰)
        double cos_phi = std::cos(phi);
        double sin_phi = std::sin(phi);

        for (uint16_t i = 0; i < lidar.horiz_res; ++i) {
            double theta = h_angles[i];        // 水平角 (方位)
            double r = lidar.ranges[static_cast<size_t>(j) * lidar.horiz_res + i];

            // 过滤无效点 (0 / 无穷 / NaN)
            if (!std::isfinite(r) || r <= 0.0f)
                continue;

            // 球坐标 → 笛卡尔 (LiDAR 坐标系: X前, Y左, Z上)
            double x = r * cos_phi * std::sin(theta);
            double y = r * cos_phi * std::cos(theta);
            double z = r * sin_phi;

            points.push_back({x, y, z});
        }
    }

    points.shrink_to_fit();
    return points;
}


// ============================================================================
//  主循环
// ============================================================================

int main(int argc, char* argv[]) {
    // ── 命令行参数 ────────────────────────────────────────
    std::string host = proto::DEFAULT_HOST;
    uint16_t    port = proto::DEFAULT_PORT;

    for (int i = 1; i < argc; ++i) {
        std::string_view arg{argv[i]};
        if (arg == "--host" && i + 1 < argc) host = argv[++i];
        else if (arg == "--port" && i + 1 < argc) port = static_cast<uint16_t>(std::stoi(argv[++i]));
        else {
            std::println(stderr, "用法: {} [--host HOST] [--port PORT]", argv[0]);
            std::println(stderr, "  默认 host={} port={}", host, port);
            return 1;
        }
    }

    // ── 主重连循环 (仿真中途重启也不怕) ────────────────────
    while (true) {
        try {
            std::println(stderr, "[WSL] Connecting to {}:{} ...", host, port);

            TcpClient client;
            client.connect(host, port);
            std::println(stderr, "[WSL] Connected!");

            // ── 读取首条消息: 应当是 0xFF 元信息 ───────────
            std::array<uint8_t, proto::HEADER_SIZE> header_buf;
            client.recv_exact(std::span<uint8_t, proto::HEADER_SIZE>{header_buf});

            auto hdr = proto::parse_header(header_buf);
            std::vector<uint8_t> payload(hdr.payload_len);
            client.recv_exact(payload);

            SensorInfo sensor_info;
            if (hdr.type == proto::INFO) {
                std::string_view json_str(reinterpret_cast<const char*>(payload.data()), payload.size());
                std::println(stderr, "[WSL] Metadata (raw): {}",
                             json_str.substr(0, 300));  // 只打印前 300 字符
                sensor_info = parse_metadata(json_str);
            }

            // ── 主接收循环 ────────────────────────────────
            int    frame_count  = 0;
            size_t last_lidar_pts = 0;
            int    last_cam_w  = 0;
            int    last_cam_h  = 0;
            auto   last_tick   = std::chrono::steady_clock::now();

            while (true) {
                // 1. 读头部
                client.recv_exact(std::span<uint8_t, proto::HEADER_SIZE>{header_buf});
                hdr = proto::parse_header(header_buf);

                // 2. 读负载
                payload.resize(hdr.payload_len);
                client.recv_exact(payload);

                // 3. 按类型分发
                switch (hdr.type) {
                case proto::CAMERA: {
                    auto cam = parse_camera(payload);
                    last_cam_w = cam.width;
                    last_cam_h = cam.height;
                    // 你的 SLAM 算法在这里获取 cam.image (BGRA uint8)
                    //
                    // 示例: 转灰度 (亮度 = 0.299R + 0.587G + 0.114B)
                    // 但注意 Webots 是 BGRA 顺序:
                    //   for (size_t i = 0; i < cam.image.size(); i += 4) {
                    //       uint8_t b = cam.image[i];
                    //       uint8_t g = cam.image[i+1];
                    //       uint8_t r = cam.image[i+2];
                    //       uint8_t gray = uint8_t(0.299*r + 0.587*g + 0.114*b);
                    //   }
                    break;
                }
                case proto::LIDAR: {
                    auto lidar = parse_lidar(payload);
                    auto cloud = lidar_to_pointcloud(lidar,
                                                     sensor_info.lidar_fov,
                                                     sensor_info.lidar_vfov);
                    last_lidar_pts = cloud.size();
                    // 你的 SLAM 算法在这里获取 cloud (std::vector<Point3D>)
                    //
                    // 示例: 访问第 i 个点
                    //   double x = cloud[i].x;
                    //   double y = cloud[i].y;
                    //   double z = cloud[i].z;
                    break;
                }
                case proto::INFO: {
                    std::string_view js(reinterpret_cast<const char*>(payload.data()), payload.size());
                    sensor_info = parse_metadata(js);
                    std::println(stderr, "[WSL] Metadata updated");
                    break;
                }
                default:
                    std::println(stderr, "[WSL] Unknown message type: 0x{:02x}", hdr.type);
                    break;
                }

                // 4. 每秒统计输出
                ++frame_count;
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_tick).count();
                if (elapsed >= 1) {
                    std::println(stderr, "[WSL] {} msgs/s | last cam = {}x{} | last lidar = {} pts",
                                 frame_count / (elapsed ? elapsed : 1),
                                 last_cam_w, last_cam_h, last_lidar_pts);
                    frame_count = 0;
                    last_tick   = now;
                }
            }

        } catch (const std::exception& e) {
            std::println(stderr, "[WSL] Error: {}", e.what());
            std::println(stderr, "[WSL] Reconnecting in 2 seconds...");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // 自动重试 (外层的 while(true) )
        }
    }

    return 0;  // unreachable
}
