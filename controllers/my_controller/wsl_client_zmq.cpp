// ============================================================================
// wsl_client_zmq.cpp — C++23 ZeroMQ 版 WSL 传感器数据订阅客户端
//
// 相比裸 TCP 版的差异:
//   ✓ 不需要自己处理粘包/拆包 (ZMQ 自带消息边界)
//   ✓ 不需要手动重连逻辑 (ZMQ 自动重连)
//   ✓ 内置 topic 过滤, 可只订阅需要的传感器
//   ✓ 可以随时追加第二个消费者 (录盘/可视化) 而不用改发布者
//
// 编译:
//   g++-16 -std=c++23 -O2 -o wsl_client_zmq wsl_client_zmq.cpp -lzmq
//
// 运行:
//   ./wsl_client_zmq                    (默认 127.0.0.1:12345)
//   ./wsl_client_zmq --host 172.x.x.x   (指定 Windows IP)
//
// 安装依赖 (WSL):
//   sudo apt install libzmq3-dev libcppzmq-dev
// ============================================================================

#include <print>                     // C++23
#include <cstdint>
#include <vector>
#include <string>
#include <cstring>
#include <string_view>
#include <cmath>
#include <chrono>
#include <thread>
#include <expected>
#include <optional>
#include <array>
#include <span>

// ── ZeroMQ (C++ bindings, header-only) ──────────────────────────────────
#include <zmq.hpp>       // 核心: zmq::context_t, zmq::socket_t, zmq::message_t

// ============================================================================
//  字节序工具 (与 Python 端 struct.pack("!...") 一致: 大端序)
// ============================================================================

namespace byte_util {

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
    [[nodiscard]] float read_be_float(const uint8_t* p) noexcept {
        uint32_t bits = read_be32(p);
        float result;
        std::memcpy(&result, &bits, sizeof(result));
        return result;
    }

} // namespace byte_util


// ============================================================================
//  传感器数据结构
// ============================================================================

struct CameraFrame {
    uint64_t           timestamp_us;
    uint16_t           width;
    uint16_t           height;
    std::vector<uint8_t> image;  // BGRA, size = width * height * 4
};

struct LidarFrame {
    uint64_t           timestamp_us;
    uint16_t           num_layers;
    uint16_t           horiz_res;
    std::vector<float> ranges;   // size = num_layers * horiz_res
};

struct Point3D { double x, y, z; };

struct SensorInfo {
    double lidar_fov  = 3.0;
    double lidar_vfov = 1.03;
};


// ============================================================================
//  元信息 JSON 极简解析
// ============================================================================

[[nodiscard]] static
std::optional<double> json_extract_double(std::string_view json, std::string_view key) {
    auto key_start = json.find(key);
    if (key_start == std::string_view::npos) return std::nullopt;
    auto colon = json.find(':', key_start + key.size());
    if (colon == std::string_view::npos) return std::nullopt;
    auto val_start = json.find_first_not_of(" \t\n\r", colon + 1);
    if (val_start == std::string_view::npos) return std::nullopt;
    auto val_end = val_start;
    while (val_end < json.size() && json[val_end] != ','
           && json[val_end] != '}' && json[val_end] != ']'
           && json[val_end] != '\n' && json[val_end] != ' ') ++val_end;
    try {
        return std::stod(std::string(json.substr(val_start, val_end - val_start)));
    } catch (...) { return std::nullopt; }
}

[[nodiscard]] static
SensorInfo parse_metadata(std::string_view json) {
    SensorInfo info;
    auto lidar_pos = json.find("\"lidar\"");
    if (lidar_pos == std::string_view::npos) return info;
    auto block = json.substr(lidar_pos);
    if (auto f = json_extract_double(block, "\"fov\""))          info.lidar_fov  = *f;
    if (auto f = json_extract_double(block, "\"vertical_fov\"")) info.lidar_vfov = *f;
    std::println(stderr, "[ZMQ]  LiDAR FOV: {:.3f} rad (H) × {:.3f} rad (V)",
                 info.lidar_fov, info.lidar_vfov);
    return info;
}


// ============================================================================
//  反序列化: bytes → 结构化数据
// ============================================================================
//  ZMQ 版本的消息 payload 格式 (多部分消息的第二部分):
//
//  相机 "camera":  ts_us(8B) + width(2B) + height(2B) + BGRA pixels(NB)
//  LiDAR "lidar":  ts_us(8B) + num_layers(2B) + horiz_res(2B) + ranges(NB)

[[nodiscard]] CameraFrame parse_camera(std::span<const uint8_t> payload) {
    if (payload.size() < 12)
        throw std::runtime_error("Camera payload too short");

    uint64_t ts  = byte_util::read_be64(payload.data());
    uint16_t w   = byte_util::read_be16(payload.data() + 8);
    uint16_t h   = byte_util::read_be16(payload.data() + 10);

    size_t img_size = static_cast<size_t>(w) * h * 4;
    if (payload.size() != 12 + img_size)
        throw std::runtime_error("Camera payload size mismatch");

    return CameraFrame{
        .timestamp_us = ts,
        .width  = w,
        .height = h,
        .image  = std::vector<uint8_t>(payload.begin() + 12, payload.end()),
    };
}

[[nodiscard]] LidarFrame parse_lidar(std::span<const uint8_t> payload) {
    if (payload.size() < 12)
        throw std::runtime_error("Lidar payload too short");

    uint64_t ts    = byte_util::read_be64(payload.data());
    uint16_t layers= byte_util::read_be16(payload.data() + 8);
    uint16_t hres  = byte_util::read_be16(payload.data() + 10);

    size_t n_pts = static_cast<size_t>(layers) * hres;
    size_t expected = 12 + n_pts * 4;   // 12字节头部 + N个float32
    if (payload.size() != expected)
        throw std::runtime_error("Lidar payload size mismatch");

    std::vector<float> ranges(n_pts);
    for (size_t i = 0; i < n_pts; ++i) {
        ranges[i] = byte_util::read_be_float(payload.data() + 12 + i * 4);
    }

    return LidarFrame{
        .timestamp_us = ts,
        .num_layers = layers,
        .horiz_res  = hres,
        .ranges     = std::move(ranges),
    };
}


// ============================================================================
//  LiDAR 距离 → 3D 点云
// ============================================================================

[[nodiscard]] std::vector<Point3D> lidar_to_pointcloud(
    const LidarFrame& lidar, double fov, double vfov)
{
    std::vector<double> h_angles(lidar.horiz_res);
    std::vector<double> v_angles(lidar.num_layers);

    for (uint16_t i = 0; i < lidar.horiz_res; ++i)
        h_angles[i] = -fov / 2.0 + (double(i) + 0.5) * fov / lidar.horiz_res;
    for (uint16_t j = 0; j < lidar.num_layers; ++j)
        v_angles[j] = -vfov / 2.0 + (double(j) + 0.5) * vfov / lidar.num_layers;

    std::vector<Point3D> points;
    points.reserve(lidar.ranges.size());

    for (uint16_t j = 0; j < lidar.num_layers; ++j) {
        double phi = v_angles[j];
        double cos_phi = std::cos(phi);
        double sin_phi = std::sin(phi);
        for (uint16_t i = 0; i < lidar.horiz_res; ++i) {
            double theta = h_angles[i];
            double r = lidar.ranges[static_cast<size_t>(j) * lidar.horiz_res + i];
            if (!std::isfinite(r) || r <= 0.0) continue;
            points.push_back({
                r * cos_phi * std::sin(theta),   // X: 向前
                r * cos_phi * std::cos(theta),   // Y: 向左
                r * sin_phi                      // Z: 向上
            });
        }
    }
    return points;
}


// ============================================================================
//  主函数
// ============================================================================

int main(int argc, char* argv[]) {
    // ── 命令行参数 ────────────────────────────────────────
    std::string host = "127.0.0.1";
    uint16_t    port = 12345;

    for (int i = 1; i < argc; ++i) {
        std::string_view arg{argv[i]};
        if (arg == "--host" && i + 1 < argc) host = argv[++i];
        else if (arg == "--port" && i + 1 < argc) port = uint16_t(std::stoi(argv[++i]));
        else {
            std::println(stderr, "用法: {} [--host HOST] [--port PORT]", argv[0]);
            return 1;
        }
    }

    // ── ZMQ 初始化 ────────────────────────────────────────
    zmq::context_t context(1);

    // SUB 套接字: 连接发布者, 订阅所有 topic
    zmq::socket_t subscriber(context, zmq::socket_type::sub);
    subscriber.set(zmq::sockopt::rcvhwm, 4);    // 最多缓冲 4 帧
    subscriber.set(zmq::sockopt::linger, 0);     // 退出时丢弃未处理消息

    // 关键: 设置订阅前缀。"" 空字符串 = 订阅全部
    // 也可以精细控制: "camera" = 只收相机; "lidar" = 只收雷达
    subscriber.set(zmq::sockopt::subscribe, "");

    std::string endpoint = "tcp://" + host + ":" + std::to_string(port);
    subscriber.connect(endpoint);
    std::println(stderr, "[ZMQ] SUB connected to {}", endpoint);

    // ── 等待并接收首条元信息 ──────────────────────────────
    // ZMQ SUB 有 "slow joiner" 现象: 刚 connect 后的前几毫秒可能丢消息
    // 发布端已经做了 sleep(1) + 周期性重发, 所以这里不需要额外等待
    std::println(stderr, "[ZMQ] Waiting for first info message...");

    SensorInfo sensor_info;
    bool got_info = false;

    // 最多等 3 秒拿元信息, 拿不到就 fallback
    {
        zmq::pollitem_t poll_items[] = {
            { static_cast<void*>(subscriber), 0, ZMQ_POLLIN, 0 }
        };

        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (!got_info && std::chrono::steady_clock::now() < deadline) {
            int rc = zmq::poll(poll_items, 1, std::chrono::milliseconds(200));
            if (rc > 0 && (poll_items[0].revents & ZMQ_POLLIN)) {
                zmq::message_t topic, payload;
                subscriber.recv(topic);
                subscriber.recv(payload);

                std::string_view topic_str(
                    static_cast<const char*>(topic.data()), topic.size());

                if (topic_str == "info") {
                    std::string_view json_str(
                        static_cast<const char*>(payload.data()), payload.size());
                    sensor_info = parse_metadata(json_str);
                    got_info = true;
                    std::println(stderr, "[ZMQ] Initial metadata received");
                }
            }
        }
    }

    if (!got_info)
        std::println(stderr, "[ZMQ] No metadata within 3s, using defaults");

    // ── 主接收循环 ────────────────────────────────────────
    int    frame_count   = 0;
    size_t last_pts      = 0;
    int    last_cam_w    = 0, last_cam_h = 0;
    auto   last_tick     = std::chrono::steady_clock::now();

    try {
        while (true) {
            // ZMQ 多部分消息接收:
            //   第1部分: topic (字符串)
            //   第2部分: payload (二进制)
            zmq::message_t topic_msg, payload_msg;

            // recv 返回时, 消息总是完整的——ZMQ 保证了这一点
            // 不会出现"只收到半个相机帧"的情况
            subscriber.recv(topic_msg);
            subscriber.recv(payload_msg);

            std::string_view topic(
                static_cast<const char*>(topic_msg.data()), topic_msg.size());

            auto payload_bytes = std::span<const uint8_t>(
                static_cast<const uint8_t*>(payload_msg.data()), payload_msg.size());

            if (topic == "camera") {
                auto cam = parse_camera(payload_bytes);
                last_cam_w = cam.width;
                last_cam_h = cam.height;
                // 你的 SLAM 算法在这里使用 cam.image
                // 内存布局: BGRA, 行优先, (height × width × 4) bytes
                // 注意: cam.image 在解析时复制了一份, 可以安全传出

            } else if (topic == "lidar") {
                auto lidar = parse_lidar(payload_bytes);
                auto cloud = lidar_to_pointcloud(lidar,
                    sensor_info.lidar_fov, sensor_info.lidar_vfov);
                last_pts = cloud.size();
                // 你的 SLAM 算法在这里使用 cloud (std::vector<Point3D>)
                // cloud[i].x, cloud[i].y, cloud[i].z

            } else if (topic == "info") {
                std::string_view json_str(
                    static_cast<const char*>(payload_msg.data()), payload_msg.size());
                sensor_info = parse_metadata(json_str);
                std::println(stderr, "[ZMQ] Metadata updated");

            } else {
                std::println(stderr, "[ZMQ] Unknown topic: {}", topic);
            }

            // 每秒统计
            ++frame_count;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                               now - last_tick).count();
            if (elapsed >= 1) {
                std::println(stderr, "[ZMQ] {} msgs/s | cam = {}x{} | lidar = {} pts",
                             frame_count, last_cam_w, last_cam_h, last_pts);
                frame_count = 0;
                last_tick   = now;
            }
        }
    } catch (const zmq::error_t& e) {
        std::println(stderr, "[ZMQ] ZMQ error: {} (errno={})", e.what(), e.num());
    } catch (const std::exception& e) {
        std::println(stderr, "[ZMQ] Error: {}", e.what());
    }

    // ZMQ 的 RAII 析构会自动关闭 socket + 终止上下文, 无需手动处理
    std::println(stderr, "[ZMQ] Shutdown");
    return 0;
}
