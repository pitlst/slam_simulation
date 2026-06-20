"""
Webots SLAM 仿真控制器 — ZeroMQ PUB 版本
=========================================
替代方案: 用 ZMQ PUB/SUB 代替裸 TCP, 提供消息边界 + 多消费者扇出.

如需切回裸 TCP 版, 编辑 world 文件将 controller 改回 "my_controller".

安装依赖:
    pip install pyzmq

架构:
    PUB (Windows/Webots)                          SUB (WSL/算法)
    ├─ topic "camera"  ── multipart ───────→      choose your topics
    ├─ topic "lidar"   ── multipart ───────→
    └─ topic "info"    ── JSON ───────────→
"""

import sys
import traceback
import struct
import json
import time as time_module
from controller import Robot # type: ignore

try:
    import zmq
    HAS_ZMQ = True
except ImportError:
    print("[CTRL] FATAL: pyzmq not installed. Run: pip install pyzmq", file=sys.stderr)
    sys.exit(1)


# ============================================================
#  配置
# ============================================================

ZMQ_PORT = 12345
PUBLISH_INTERVAL_SEC = 32  # 每隔 N 个仿真步重发一次元信息 (供迟到订阅者)


def _make_info_dict(robot, camera, lidar) -> dict:
    """构造元信息字典 (与 TCP 版完全一致)"""
    return {
        "protocol": "1.0",
        "robot_name": robot.getName(),
        "basic_time_step_ms": int(robot.getBasicTimeStep()),
        "camera": {
            "name": camera.getName(),
            "width": camera.getWidth(),
            "height": camera.getHeight(),
            "fov": camera.getFov(),
            "near": camera.getNear(),
        },
        "lidar": {
            "name": lidar.getName(),
            "num_layers": lidar.getNumberOfLayers(),
            "horizontal_resolution": lidar.getWidth(),
            "fov": lidar.getFov(),
            "vertical_fov": lidar.getVerticalFov(),
            "min_range": lidar.getMinRange(),
            "max_range": lidar.getMaxRange(),
        },
        "extrinsics_camera_to_lidar": {
            "translation": [0, 0, 0],
            "rotation": [0, 0, 1, 0],
        },
    }


def _make_camera_msg(image_bytes: bytes, width: int, height: int, ts_us: int) -> bytes:
    """
    相机消息 payload:
        ts_us(8B, big-endian) + width(2B) + height(2B) + BGRA pixels(NB)
    """
    return struct.pack("!QHH", ts_us, width, height) + image_bytes


def _make_lidar_msg(ranges: list, num_layers: int, horiz_res: int, ts_us: int) -> bytes:
    """
    LiDAR 消息 payload:
        ts_us(8B) + num_layers(2B) + horiz_res(2B) + float32 ranges(NB)
    """
    header = struct.pack("!QHH", ts_us, num_layers, horiz_res)
    fmt = f"{len(ranges)}f"
    return header + struct.pack(fmt, *ranges)


# ============================================================
#  主循环
# ============================================================

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    print(f"[CTRL] Controller (ZMQ) started, timestep={timestep}ms", file=sys.stderr)

    # ── 获取设备 ──────────────────────────────────────────
    camera = robot.getDevice("海康CA01610UC")
    lidar  = robot.getDevice("览沃Mid360")
    if not camera or not lidar:
        print("[CTRL] ERROR: devices not found", file=sys.stderr)
        sys.exit(1)

    camera.enable(timestep)
    lidar.enable(timestep)
    if robot.step(timestep) == -1:
        return

    cam_w, cam_h = camera.getWidth(), camera.getHeight()
    lidar_layers = lidar.getNumberOfLayers()
    lidar_width  = lidar.getWidth()
    print(f"[CTRL] Camera: {cam_w}x{cam_h}", file=sys.stderr)
    print(f"[CTRL] LiDAR:  {lidar_layers} layers × {lidar_width} pts",
          file=sys.stderr)

    # ── ZMQ PUB 套接字 ────────────────────────────────────
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.SNDHWM, 4)   # 最多缓冲 4 帧, 防内存堆积
    publisher.setsockopt(zmq.LINGER, 0)   # 退出时立即丢弃未发送消息
    publisher.bind(f"tcp://*:{ZMQ_PORT}")
    print(f"[CTRL] ZMQ PUB bound to tcp://*:{ZMQ_PORT}", file=sys.stderr)

    # ── ZMQ "Slow Joiner" 问题 ────────────────────────────
    # 在 PUB 上 bind() 之后马上 send(), 刚连接的 SUB 可能收不到
    # 前几条消息。这是 ZMQ 内部传播 subscription 需要时间。
    # 解决: 短暂等待 + 周期性重发 info 消息。
    info_dict = _make_info_dict(robot, camera, lidar)
    info_bytes = json.dumps(info_dict, ensure_ascii=False).encode("utf-8")

    time_module.sleep(1.0)  # 给潜在订阅者连接时间
    publisher.send_multipart([b"info", info_bytes])
    print("[CTRL] Info published (initial)", file=sys.stderr)

    # ── 仿真主循环 ────────────────────────────────────────
    step_count = 0
    try:
        while robot.step(timestep) != -1:
            ts_us = int(robot.getTime() * 1_000_000)
            step_count += 1

            # ── 相机 (一次性发送多部分消息) ──────────────
            img = camera.getImage()
            if img is not None:
                cam_payload = _make_camera_msg(img, cam_w, cam_h, ts_us)
                publisher.send_multipart([b"camera", cam_payload],
                                         zmq.NOBLOCK)
                # NOBLOCK: 如果 subscriber 处理太慢导致 HWM 达到,
                # 直接丢弃这一帧而不是阻塞仿真。对 SLAM 来说,
                # 跳一帧比拖慢整个仿真要好。

            # ── LiDAR ─────────────────────────────────────
            ranges = lidar.getRangeImage()
            if ranges is not None:
                lid_payload = _make_lidar_msg(ranges, lidar_layers,
                                              lidar_width, ts_us)
                publisher.send_multipart([b"lidar", lid_payload],
                                         zmq.NOBLOCK)

            # ── 周期性重发元信息 (供迟到订阅者) ──────────
            if step_count % PUBLISH_INTERVAL_SEC == 0:
                try:
                    publisher.send_multipart([b"info", info_bytes],
                                             zmq.NOBLOCK)
                except zmq.ZMQError:
                    pass

    except KeyboardInterrupt:
        print("[CTRL] Stopped by user", file=sys.stderr)
    except Exception as e:
        print(f"[CTRL] FATAL: {e}", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
    finally:
        publisher.close(linger=0)
        context.term()
        print("[CTRL] ZMQ context terminated", file=sys.stderr)


if __name__ == "__main__":
    main()
