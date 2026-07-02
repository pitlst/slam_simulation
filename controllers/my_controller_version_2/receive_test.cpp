// ============================================================
// ① 引入头文件
// ============================================================
#include <zmq.hpp>          // cppzmq 封装库：提供 C++ 风格的 ZeroMQ 类（zmq::context_t, zmq::socket_t 等）
#include <zstd.h>           // Zstandard 官方 C API：提供 ZSTD_createDCtx / ZSTD_decompressDCtx 等函数
#include <iostream>         // 标准输入输出流：std::cout, std::cerr, std::endl
#include <iomanip>          // 格式化输出：std::fixed, std::setprecision
#include <vector>           // 动态数组容器：std::vector<uint8_t>
#include <chrono>           // 时间库：std::chrono::steady_clock, std::chrono::duration
#include <thread>           // 线程支持（虽然这里没显式创建线程，但 zmq 内部可能用到）
#include <cstring>          // C 风格内存操作：std::memcpy, std::memcmp
#include <csignal>          // 信号处理：std::signal, SIGINT
#include <atomic>           // 原子变量：std::atomic<bool>，保证多线程/信号下的可见性

// ② 字节序检查
// __BYTE_ORDER__ 是 GCC/Clang 内置宏，表示当前编译目标平台的字节序
// __ORDER_LITTLE_ENDIAN__ 表示小端（x86/x64 都是小端）
// Python 的 struct.unpack("<...") 中 "<" 代表"小端、无对齐"
// 因此如果主机是大端，memcpy 出来的数值会和 Python 不一致，需要额外转换
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
              "Host must be little-endian to match Python struct format");

// ③ OpenCV 条件编译开关
// 如果定义了 WITH_OPENCV 宏，才会编译 #include 到 #endif 之间的代码
// 这样即使系统没装 OpenCV，也能正常编译（只是不显示图像）
// #include <opencv2/opencv.hpp>  // OpenCV 主头文件，包含 cv::Mat, cv::imshow, cv::waitKey 等

// ============================================================
// ④ 全局原子标志：用于优雅地响应 Ctrl+C（SIGINT）
// ============================================================
// std::atomic<bool> 保证对 bool 的读写是原子的，避免编译器优化导致的可见性问题
// g_running 初始值为 true，收到 SIGINT 后信号处理函数会把它设为 false
std::atomic<bool> g_running{true};

// ⑤ 信号处理函数
// 参数 int：信号编号，这里用不到，但函数签名必须匹配 void(*)(int)
// 当用户在终端按 Ctrl+C 时，操作系统会发送 SIGINT 信号给进程
void signal_handler(int) {
    g_running = false;  // 把全局标志置为 false，主循环 while(g_running) 就会退出
}

// ============================================================
// ⑥ 帧率统计结构体
// 对应 Python 的 FPS_STATS = {b"camera": 0, b"lidar": 0, b"accel": 0, b"gyro": 0}
// ============================================================
struct FPSStats {
    int camera = 0;  // 累计收到的 camera 消息数量
    int lidar  = 0;  // 累计收到的 lidar 消息数量
    int accel  = 0;  // 累计收到的 accel 消息数量
    int gyro   = 0;  // 累计收到的 gyro 消息数量
};

// ============================================================
// ⑦ 主函数入口
// ============================================================
int main() {
    // ⑧ 注册信号处理函数
    // 第一个参数 SIGINT：中断信号（通常由 Ctrl+C 产生）
    // 第二个参数 signal_handler：上面定义的回调函数指针
    // 返回值是旧的处理函数指针，这里忽略
    std::signal(SIGINT, signal_handler);

    // ⑨ 创建 ZMQ 上下文
    // zmq::context_t 对应 Python 的 zmq.Context()
    // 参数 1：IO 线程数量，一般 1 就够了，负责后台网络收发
    zmq::context_t context(1);

    // ⑩ 创建 SUB（订阅者）套接字
    // zmq::socket_type::sub 对应 Python 的 zmq.SUB
    // 该套接字只能接收消息，不能发送
    zmq::socket_t socket(context, zmq::socket_type::sub);

    // ⑪ 连接到发布端
    // "tcp://localhost:5555" 是端点地址，与 Python 完全一致
    // 这里 socket 是客户端角色，主动 connect 到服务器的 5555 端口
    socket.connect("tcp://localhost:5555");

    // ⑫ 设置订阅过滤器
    // 空字符串 "" 表示订阅所有主题（对应 Python 的 socket.setsockopt(zmq.SUBSCRIBE, b"")）
    // 如果只想订阅 camera，可改为 "camera"
    // zmq::sockopt::subscribe 是 cppzmq 对 ZMQ_SUBSCRIBE 的 C++ 封装
    socket.set(zmq::sockopt::subscribe, "");

    // ⑬ 设置接收超时时间
    // 参数 100：单位毫秒（ms），对应 Python 的 socket.setsockopt(zmq.RCVTIMEO, 100)
    // 如果 100ms 内没有数据到达，recv() 会抛出 zmq::error_t，错误码 EAGAIN
    // 这样主循环不会永远阻塞，可以定期检查 g_running 和打印帧率
    socket.set(zmq::sockopt::rcvtimeo, 100);

    // ⑭ 创建 Zstandard 解压缩上下文
    // ZSTD_createDCtx() 对应 Python 的 zstandard.ZstdDecompressor()
    // 返回 ZSTD_DCtx* 指针，内部维护了字典、状态等解压缩所需资源
    // 复用同一个 DCtx 比每次都创建销毁更高效
    ZSTD_DCtx* dctx = ZSTD_createDCtx();
    if (!dctx) {
        // 如果内存分配失败，dctx 为 nullptr
        std::cerr << "Failed to create ZSTD decompression context" << std::endl;
        return 1;  // 返回非 0 表示程序异常退出
    }

    // ⑮ 帧率统计变量
    // stats 是 FPSStats 结构体实例，记录当前这一秒内各主题的消息数
    FPSStats stats;
    // last_print 记录上次打印帧率的时间点，用于判断是否已经过了 1 秒
    auto last_print = std::chrono::steady_clock::now();
    // decompress_buf 是复用的输出缓冲区，避免每次解压缩都重新分配内存
    // 初始为空，第一次收到 camera 消息时按实际大小 resize
    std::vector<uint8_t> decompress_buf;

    // ⑯ 提示信息，对应 Python 的 logger.info("订阅端已连接...")
    std::cout << "订阅端已连接，等待 Webots 数据..." << std::endl;

    // ⑰ 条件编译：如果定义了 WITH_OPENCV，创建 OpenCV 窗口
    // cv::WINDOW_NORMAL 表示窗口可调整大小，对应 Python 的 cv2.WINDOW_NORMAL
    // cv::namedWindow("Camera Frame", cv::WINDOW_NORMAL);

    // ============================================================
    // ⑱ 主循环，对应 Python 的 while True:
    // 条件 g_running 初始为 true，收到 SIGINT 后变为 false，循环退出
    // ============================================================
    try {
        while (g_running) {
            // ⑲ 获取当前时间点
            // steady_clock 是单调时钟，不受系统时间调整影响，适合测量时间间隔
            auto now = std::chrono::steady_clock::now();

            // ⑳ 计算距离上次打印帧率经过了多少秒
            // duration<double> 会把时间差转换为以秒为单位的浮点数
            double elapsed = std::chrono::duration<double>(now - last_print).count();

            // ㉑ 如果已经过了一秒（或更久），打印并清零统计
            // 对应 Python 的 if now - FPS_LAST_PRINT_TIME >= 1.0:
            if (elapsed >= 1.0) {
                // std::fixed 表示固定小数位，std::setprecision(1) 保留 1 位小数
                // 对应 Python 的 f"{cam:.1f}"
                std::cout << "[Recv FPS] camera=" << std::fixed << std::setprecision(1)
                          << (stats.camera / elapsed)   // 消息数 / 秒数 = 频率
                          << " | lidar=" << (stats.lidar / elapsed)
                          << " | accel=" << (stats.accel / elapsed)
                          << " | gyro=" << (stats.gyro / elapsed) << std::endl;

                // 重置计数器，对应 Python 的 FPS_STATS = {b"camera": 0, ...}
                stats.camera = 0;
                stats.lidar = 0;
                stats.accel = 0;
                stats.gyro = 0;

                // 更新上次打印时间为当前时间，开启下一个 1 秒统计周期
                last_print = now;
            }

            // ============================================================
            // ㉒ 接收 multipart 消息的第一部分：topic
            // Python 的 topic, payload = socket.recv_multipart()
            // 在 C++ 中需要分两次接收，并检查是否有更多帧
            // ============================================================
            zmq::message_t topic_msg;   // zmq::message_t 是 ZMQ 的消息缓冲区类，自动管理内存
            zmq::recv_result_t result;  // recv_result_t 是 size_t 的 optional 包装，表示实际接收到的字节数

            // ㉓ 尝试接收 topic 帧
            // zmq::recv_flags::none 表示阻塞等待（但前面设置了 100ms 超时，所以最多等 100ms）
            try {
                result = socket.recv(topic_msg, zmq::recv_flags::none);
            } catch (const zmq::error_t& e) {
                // ㉔ 如果超时，cppzmq 会抛出异常，错误码 e.num() == EAGAIN
                // 对应 Python 的 except zmq.Again: pass
                if (e.num() == EAGAIN) {
                    continue;  // 没收到数据，跳过本次循环，继续检查帧率和 g_running
                }
                // 如果是其他错误（如连接断开），重新抛出，由外层 catch 捕获
                throw;
            }

            // ㉕ result 为 std::nullopt 表示接收失败（通常也是超时）
            if (!result) {
                continue;
            }

            // ㉖ 检查是否还有后续帧（multipart 的第二帧 payload）
            // zmq::sockopt::rcvmore 对应 ZMQ_RCVMORE，返回 bool
            // 如果为 false，说明消息只有 topic 没有 payload，格式异常，丢弃
            if (!socket.get(zmq::sockopt::rcvmore)) {
                continue;
            }

            // ㉗ 接收第二帧：payload
            zmq::message_t payload_msg;
            socket.recv(payload_msg, zmq::recv_flags::none);

            // ㉘ 将 topic 的字节数据转换为 std::string，方便后续用 == 比较
            // topic_msg.data() 返回 void*，需要 static_cast 为 char*
            // topic_msg.size() 返回消息字节长度
            std::string topic(static_cast<char*>(topic_msg.data()), topic_msg.size());

            // ㉙ 获取 payload 的指针和长度
            // payload 是 const uint8_t*，表示只读的字节数组
            const uint8_t* payload = static_cast<const uint8_t*>(payload_msg.data());
            size_t payload_size = payload_msg.size();

            // ============================================================
            // ㉚ 根据 topic 分发处理，对应 Python 的 if topic == b"camera": ...
            // ============================================================

            // ==================== camera ====================
            if (topic == "camera") {
                ++stats.camera;  // 帧率计数器 +1

                // ㉛ Python 的 header_fmt = "<dIII"
                // "<" 表示小端，"d" 是 double(8字节)，"I" 是 unsigned int(4字节)
                // 因此 header 大小 = 8 + 4 + 4 + 4 = 20 字节
                constexpr size_t HDR_SZ = sizeof(double) + 3 * sizeof(uint32_t);

                // ㉜ 防御性检查：如果 payload 连 header 都不够，直接丢弃
                if (payload_size < HDR_SZ) continue;

                // ㉝ 从 payload 中解析 header 字段
                // std::memcpy(&dest, src, nbytes) 从 src 地址拷贝 nbytes 到 dest
                // 注意：这里依赖小端假设，因为 Python 用的是 "<"（小端格式）
                double timestamp;   // 时间戳，8 字节
                uint32_t w, h, c;   // 图像宽度、高度、通道数，各 4 字节
                std::memcpy(&timestamp, payload, sizeof(double));          // 偏移 0
                std::memcpy(&w, payload + 8, sizeof(uint32_t));            // 偏移 8
                std::memcpy(&h, payload + 12, sizeof(uint32_t));             // 偏移 12
                std::memcpy(&c, payload + 16, sizeof(uint32_t));             // 偏移 16

                // ㉞ Python 的 len_fmt = "<II"  → 2 个 uint32_t
                // compressed_len: 压缩后数据的字节长度
                // raw_len: 解压缩后的原始数据字节长度（即图像大小 = h * w * c）
                constexpr size_t LEN_SZ = 2 * sizeof(uint32_t);
                if (payload_size < HDR_SZ + LEN_SZ) continue;

                uint32_t compressed_len, raw_len;
                std::memcpy(&compressed_len, payload + HDR_SZ, sizeof(uint32_t));      // 偏移 20
                std::memcpy(&raw_len, payload + HDR_SZ + 4, sizeof(uint32_t));         // 偏移 24

                // ㉟ 计算压缩数据的起始偏移和结束位置
                size_t data_offset = HDR_SZ + LEN_SZ;  // 20 + 8 = 28
                if (payload_size < data_offset + compressed_len) continue;

                // ㊱ 确保输出缓冲区足够大
                // 如果 raw_len 比当前缓冲区大，就 resize 到 raw_len
                // 如果比当前小，则复用已有内存，避免频繁分配
                if (decompress_buf.size() < raw_len) {
                    decompress_buf.resize(raw_len);
                }

                // ㊲ 执行 Zstandard 解压缩
                // ZSTD_decompressDCtx 参数说明：
                //   dctx: 解压缩上下文（前面创建的）
                //   decompress_buf.data(): 输出缓冲区指针
                //   raw_len: 输出缓冲区大小（期望解压缩后的长度）
                //   payload + data_offset: 输入（压缩数据）指针
                //   compressed_len: 输入压缩数据的长度
                // 返回值 dsize：实际解压缩后的字节数
                size_t dsize = ZSTD_decompressDCtx(
                    dctx,
                    decompress_buf.data(), raw_len,
                    payload + data_offset, compressed_len
                );

                // ㊳ 校验解压缩结果
                // 如果 dsize != raw_len，说明数据损坏或格式不匹配
                if (dsize != raw_len) {
                    std::cerr << "ZSTD decompression failed: expected " << raw_len
                              << ", got " << dsize << std::endl;
                    continue;  // 丢弃这一帧，继续接收下一帧
                }

                // ㊴ 此时 decompress_buf 中就是原始图像数据，排列为 HWC 格式
                // 即先行、后列、最后通道，与 Python 的 np.reshape((h, w, c)) 一致
                // 数据类型为 uint8_t，对应 Python 的 np.uint8

                // ㊵ 条件编译：如果启用了 OpenCV，显示图像
                // cv::Mat 构造函数参数：(rows, cols, type, data_pointer)
                // rows = h, cols = w, type 根据通道数 c 决定：
                //   c==3 → CV_8UC3（BGR 三通道）
                //   c==1 → CV_8UC1（灰度单通道）
                //   其他 → CV_8UC(c)（通用多通道）
                // cv::imshow("Camera Frame", img);  // 显示到前面 namedWindow 创建的窗口
            }
            // ==================== lidar ====================
            else if (topic == "lidar") {
                ++stats.lidar;

                // ㊶ Python 的 struct.unpack_from("<dI", payload, 0)
                // 偏移 0：double(8字节) 时间戳
                // 偏移 8：uint32_t(4字节) 点云数量
                if (payload_size < sizeof(double) + sizeof(uint32_t)) continue;

                double timestamp;
                uint32_t n_points;
                std::memcpy(&timestamp, payload, sizeof(double));
                std::memcpy(&n_points, payload + sizeof(double), sizeof(uint32_t));

                // ㊷ 剩余字节就是点云数据，每个点由 3 个 float 组成（x, y, z）
                // 因此总 float 数 = 剩余字节数 / sizeof(float)
                size_t points_bytes = payload_size - sizeof(double) - sizeof(uint32_t);
                const float* points = reinterpret_cast<const float*>(
                    payload + sizeof(double) + sizeof(uint32_t)
                );
                size_t n_floats = points_bytes / sizeof(float);

                // ㊸ (void)var; 是消除"未使用变量"编译警告的技巧
                // 这里只是演示解析，实际使用时可以存入点云数据结构
                (void)n_points; (void)points; (void)n_floats;
            }
            // ==================== accel ====================
            else if (topic == "accel") {
                ++stats.accel;

                // ㊹ Python 的 struct.unpack("<dddd", payload)
                // 4 个 double，连续排列：timestamp, x, y, z
                // 总长度 = 4 * 8 = 32 字节
                if (payload_size < 4 * sizeof(double)) continue;

                double timestamp, x, y, z;
                std::memcpy(&timestamp, payload, sizeof(double));      // 偏移 0
                std::memcpy(&x, payload + 8, sizeof(double));          // 偏移 8
                std::memcpy(&y, payload + 16, sizeof(double));          // 偏移 16
                std::memcpy(&z, payload + 24, sizeof(double));          // 偏移 24

                // 如需使用加速度数据，可在此添加业务逻辑
                (void)timestamp; (void)x; (void)y; (void)z;
            }
            // ==================== gyro ====================
            else if (topic == "gyro") {
                ++stats.gyro;

                // ㊺ 与 accel 完全相同的格式："<dddd"
                if (payload_size < 4 * sizeof(double)) continue;

                double timestamp, x, y, z;
                std::memcpy(&timestamp, payload, sizeof(double));
                std::memcpy(&x, payload + 8, sizeof(double));
                std::memcpy(&y, payload + 16, sizeof(double));
                std::memcpy(&z, payload + 24, sizeof(double));

                (void)timestamp; (void)x; (void)y; (void)z;
            }

            // ㊻ 条件编译：OpenCV 键盘检测，对应 Python 的 cv2.waitKey(1)
            // waitKey(1) 等待 1ms 键盘输入，返回按键 ASCII 码
            // & 0xFF 只保留低 8 位，与 Python 一致
            // 'q' 是字符 q，27 是 ESC 键的 ASCII 码
            // if (key == 'q' || key == 27) break;  // 退出主循环
        }
    } catch (const std::exception& e) {
        // ㊼ 捕获所有标准异常（如 zmq::error_t, std::bad_alloc 等）
        // e.what() 返回异常描述字符串
        std::cerr << "Error: " << e.what() << std::endl;
    }

    // ㊽ 条件编译：关闭所有 OpenCV 窗口，释放 GUI 资源
    // cv::destroyAllWindows();

    // ㊾ 清理 Zstandard 资源
    // ZSTD_freeDCtx 释放解压缩上下文，防止内存泄漏
    ZSTD_freeDCtx(dctx);

    // ㊿ 清理 ZMQ 资源（顺序很重要：先关 socket，再关 context）
    socket.close();         // 关闭套接字，释放连接资源
    context.shutdown();     // 关闭上下文，中断所有阻塞操作
    context.close();        // 释放上下文内存，对应 Python 的 context.term()

    // 51. 程序结束提示
    std::cout << "\n关闭订阅端" << std::endl;
    return 0;  // 返回 0 表示正常退出
}