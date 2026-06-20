"""
Webots SLAM 仿真控制器
========================
功能: 从仿真环境中读取相机 + LiDAR 传感器数据,
      通过 TCP 回环接口流式传输到 WSL 中运行的 SLAM 算法核心.

数据流架构:
    Windows (Webots)  →  TCP localhost:12345  →  WSL (算法核心)

协议: 长度前缀二进制消息
    详见本文件顶部 PROTOCOL 常量说明及 wsl_client.py 示例.
"""

import sys
import traceback
import socket
import struct
import json
from controller import Robot

# ============================================================
# 协议定义 (Protocol Definition)
# ============================================================
# 每条消息的线缆格式 (所有多字节为大端序):
#
#   [0]      type:          uint8    消息类型
#   [1-8]    timestamp_us:  uint64   仿真时间戳 (微秒)
#   [9-12]   payload_len:   uint32   负载字节长度
#   [13+]    payload:       bytes    负载数据 (格式由 type 决定)
#
# 消息类型:
#   0x01 — 相机帧 (Camera Frame)
#       payload = width(uint16) + height(uint16) + image_data(N bytes)
#       image_data: BGRA 格式, 每像素 4 字节, 行优先 (top→bottom, left→right)
#       总大小 = width * height * 4
#
#   0x02 — LiDAR 点云 (LiDAR Point Cloud)
#       payload = num_layers(uint16) + horiz_resolution(uint16)
#               + range_data(N float32 values)
#       range_data: 展平的浮点距离数组 [米],
#                   按 layer0_pt0, layer0_pt1, ..., layerN_ptM 排列
#       总大小 = 4 + num_layers * horiz_resolution * 4
#
#   0xFF — 元信息 (Metadata)
#       payload = utf-8 JSON 字符串
#       连接建立后立即发送, 包含传感器标定参数等静态信息.
#
# ============================================================

PROTOCOL_VERSION = "1.0"
TCP_HOST = "0.0.0.0"  # 监听所有网络接口
TCP_PORT = 12345       # WSL 连接端口


def _pack_header(msg_type: int, timestamp_us: int, payload: bytes) -> bytes:
    """组装消息头: type(1B) + timestamp_us(8B) + payload_len(4B)"""
    header = struct.pack("!B Q I", msg_type, timestamp_us, len(payload))
    return header + payload


def _encode_camera(image_bytes: bytes, width: int, height: int, ts: int) -> bytes:
    """编码相机消息"""
    payload = struct.pack("!HH", width, height) + image_bytes
    return _pack_header(0x01, ts, payload)


def _encode_lidar(ranges: list, num_layers: int, horiz_res: int, ts: int) -> bytes:
    """编码 LiDAR 消息 — 距离值打包为 float32 大端数组"""
    fmt = f"!HH{len(ranges)}f"
    payload = struct.pack(fmt, num_layers, horiz_res, *ranges)
    return _pack_header(0x02, ts, payload)


def _make_info(robot, camera, lidar) -> bytes:
    """生成元信息消息 (传感器参数 / 标定)"""
    info = {
        "protocol": PROTOCOL_VERSION,
        "robot_name": robot.getName(),
        "basic_time_step_ms": int(robot.getBasicTimeStep()),
        "camera": {
            "name": camera.getName(),
            "width": camera.getWidth(),
            "height": camera.getHeight(),
            "fov": camera.getFov(),               # 弧度
            "near": camera.getNear(),
        },
        "lidar": {
            "name": lidar.getName(),
            "num_layers": lidar.getNumberOfLayers(),
            "horizontal_resolution": lidar.getWidth(),
            "fov": lidar.getFov(),                  # 水平视场角 (弧度)
            "vertical_fov": lidar.getVerticalFov(), # 垂直视场角 (弧度)
            "min_range": lidar.getMinRange(),
            "max_range": lidar.getMaxRange(),
        },
        # 相机 ↔ LiDAR 外参: 两者都是 Robot 的直接子节点, 相对位姿为单位矩阵
        # 如果你的 SLAM 算法需要非单位外参, 请在此处修改
        "extrinsics_camera_to_lidar": {
            "translation": [0, 0, 0],
            "rotation": [0, 0, 1, 0],  # axis-angle: 0 rad about Z
        },
    }
    payload = json.dumps(info, ensure_ascii=False).encode("utf-8")
    return _pack_header(0xFF, 0, payload)


# ============================================================
# 主循环
# ============================================================

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    print(f"[CTRL] Controller started, timestep={timestep}ms", file=sys.stderr)

    # ── 获取设备 ──────────────────────────────────────────
    camera = robot.getDevice("海康CA01610UC")
    lidar = robot.getDevice("览沃Mid360")

    if camera is None:
        print("[CTRL] ERROR: Camera '海康CA01610UC' not found!", file=sys.stderr)
        sys.exit(1)
    if lidar is None:
        print("[CTRL] ERROR: Lidar '览沃Mid360' not found!", file=sys.stderr)
        sys.exit(1)

    # ── 启用设备 ──────────────────────────────────────────
    camera.enable(timestep)
    lidar.enable(timestep)
    print("[CTRL] Devices enabled", file=sys.stderr)

    # 等待一帧让传感器初始化
    if robot.step(timestep) == -1:
        return

    cam_w, cam_h = camera.getWidth(), camera.getHeight()
    lidar_layers = lidar.getNumberOfLayers()
    lidar_width = lidar.getWidth()  # 每层水平采样点数
    print(f"[CTRL] Camera: {cam_w}x{cam_h}", file=sys.stderr)
    print(f"[CTRL] Lidar: {lidar_layers} layers × {lidar_width} pts/layer = "
          f"{lidar_layers * lidar_width} total", file=sys.stderr)

    # ── TCP 服务器 ────────────────────────────────────────
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_PORT))
    server.listen(1)
    server.setblocking(False)  # 非阻塞, 不拖慢仿真步
    print(f"[CTRL] TCP server listening on {TCP_HOST}:{TCP_PORT}", file=sys.stderr)

    client = None
    client_addr = None

    try:
        while robot.step(timestep) != -1:
            ts_us = int(robot.getTime() * 1_000_000)

            # ── 尝试接受新客户端 ──────────────────────────
            if client is None:
                try:
                    client, client_addr = server.accept()
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    client.settimeout(1.0)
                    print(f"[CTRL] Client connected: {client_addr}", file=sys.stderr)

                    # 发送元信息
                    info_msg = _make_info(robot, camera, lidar)
                    try:
                        client.sendall(info_msg)
                        print("[CTRL] Metadata sent", file=sys.stderr)
                    except (ConnectionError, OSError):
                        print("[CTRL] Client dropped during info send", file=sys.stderr)
                        client.close()
                        client = None
                        continue
                except (BlockingIOError, OSError):
                    client = None  # 没有客户端连接, 跳过
                    continue

            # ── 相机数据 ──────────────────────────────────
            img = camera.getImage()
            if img is not None:
                # img 是 bytes (每像素 4 字节 BGRA)
                frame = _encode_camera(img, cam_w, cam_h, ts_us)
                try:
                    client.sendall(frame)
                except (ConnectionError, OSError):
                    print("[CTRL] Client disconnected (camera)", file=sys.stderr)
                    client.close()
                    client = None
                    continue

            # ── LiDAR 数据 ────────────────────────────────
            ranges = lidar.getRangeImage()
            if ranges is not None:
                # ranges 是 [float, ...] 列表, 长度 = layers * width
                packet = _encode_lidar(ranges, lidar_layers, lidar_width, ts_us)
                try:
                    client.sendall(packet)
                except (ConnectionError, OSError):
                    print("[CTRL] Client disconnected (lidar)", file=sys.stderr)
                    client.close()
                    client = None
                    continue

    except KeyboardInterrupt:
        print("[CTRL] Stopped by user", file=sys.stderr)
    except Exception as e:
        print(f"[CTRL] FATAL: {e}", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
    finally:
        if client is not None:
            client.close()
        server.close()
        print("[CTRL] Server shut down", file=sys.stderr)


if __name__ == "__main__":
    main()
