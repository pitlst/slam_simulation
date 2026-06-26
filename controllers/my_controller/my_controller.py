import sys
import traceback
import struct
import json
import zmq
import time as time_module
import numpy as np
import cv2
import rerun as rr
from controller import Robot # type: ignore


# ============================================================
#  Config
# ============================================================

ZMQ_PORT = 12345
PUBLISH_INTERVAL_SEC = 32  # re-publish info every N steps (for late-joining subscribers)


def _make_info_dict(robot, camera, lidar, lidar_width: int) -> dict:
    """Build metadata info dictionary"""
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
            "horizontal_resolution": lidar_width,
            "fov": lidar.getFov(),
            "vertical_fov": lidar.getVerticalFov(),
            "min_range": lidar.getMinRange(),
            "max_range": lidar.getMaxRange(),
        },
        "imu": {
            "accelerometer": {"name": "imu_accelerometer"},
            "gyro": {"name": "imu_gyro"},
        },
        "extrinsics_camera_to_lidar": {
            "translation": [0, 0, 0],
            "rotation": [0, 0, 1, 0],
        },
    }


def _make_imu_msg(accel: list, gyro: list, ts_us: int) -> bytes:
    """
    IMU message payload:
        ts_us(8B) + ax(4B float) + ay(4B float) + az(4B float)
        + gx(4B float) + gy(4B float) + gz(4B float)
    """
    return struct.pack("!Qffffff", ts_us, *accel, *gyro)


def _make_camera_msg(image_bytes: bytes, width: int, height: int, ts_us: int) -> bytes:
    """
    Camera message payload:
        ts_us(8B, big-endian) + width(2B) + height(2B) + BGRA pixels(NB)
    """
    return struct.pack("!QHH", ts_us, width, height) + image_bytes


def _make_lidar_msg(ranges: list, num_layers: int, horiz_res: int, ts_us: int) -> bytes:
    """
    LiDAR message payload:
        ts_us(8B) + num_layers(2B) + horiz_res(2B) + float32 ranges(NB)
    """
    header = struct.pack("!QHH", ts_us, num_layers, horiz_res)
    fmt = f"{len(ranges)}f"
    return header + struct.pack(fmt, *ranges)


def _lidar_to_pointcloud(ranges: list, num_layers: int, horiz_res: int, fov: float, vfov: float) -> np.ndarray:
    """Convert LiDAR ranges to 3D point cloud (N, 3) via vectorized numpy"""
    ranges = np.asarray(ranges, dtype=np.float32).reshape(num_layers, horiz_res) # type: ignore

    # angle at each bin centre
    h_angles = np.linspace(-fov/2, fov/2, horiz_res, endpoint=False) + fov / (2 * horiz_res)
    v_angles = np.linspace(-vfov/2, vfov/2, num_layers, endpoint=False) + vfov / (2 * num_layers)

    theta = h_angles[np.newaxis, :]   # (1, horiz_res)
    phi   = v_angles[:, np.newaxis]    # (num_layers, 1)

    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)

    # Webots LiDAR coordinate system (θ=0 points along +Y):
    #   X = left,  Y = forward,  Z = up
    x_left  = ranges * cos_phi * np.sin(theta)  # +θ → +X (left)
    y_fwd   = ranges * cos_phi * np.cos(theta)  # forward
    z_up    = ranges * sin_phi                   # up

    mask = np.isfinite(ranges) & (ranges > 0) # type: ignore

    # Convert to Rerun RIGHT_HAND_Z_UP (X=right, Y=forward, Z=up):
    #   rerun_x = -left  → right
    #   rerun_y =  forward
    #   rerun_z =  up
    points = np.stack([-x_left, y_fwd, z_up], axis=-1)[mask]
    return points


# ============================================================
#  Main loop
# ============================================================

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    print(f"[CTRL] Controller (ZMQ) started, timestep={timestep}ms")

    # ── Get devices ─────────────────────────────────────────
    camera = robot.getDevice("MV-CS016-10UC-V5")
    lidar  = robot.getDevice("LivoxMid360")
    imu_accel = robot.getDevice("imu_accelerometer")
    imu_gyro  = robot.getDevice("imu_gyro")
    if not camera or not lidar or not imu_accel or not imu_gyro:
        print("[CTRL] ERROR: devices not found")
        sys.exit(1)

    camera.enable(timestep)
    lidar.enable(timestep)
    imu_accel.enable(timestep)
    imu_gyro.enable(timestep)
    if robot.step(timestep) == -1:
        return

    cam_w, cam_h = camera.getWidth(), camera.getHeight()
    lidar_layers = lidar.getNumberOfLayers()

    # Webots Python API: Lidar has no getWidth()
    # horizontal_resolution = len(range_image) // num_layers
    # Note: getRangeImage() needs enable + one step() first
    _temp_ranges = lidar.getRangeImage()
    if _temp_ranges:
        lidar_width = len(_temp_ranges) // lidar_layers
    else:
        lidar_width = 360  # fallback: matches default in .wbt
    print(f"[CTRL] Camera: {cam_w}x{cam_h}")
    print(f"[CTRL] LiDAR:  {lidar_layers} layers × {lidar_width} pts")

    # ── Rerun init ─────────────────────────────────────────
    rr.init("handheld_scanner", spawn=True)
    # RIGHT_HAND_Z_UP matches Webots default coordinate system
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    print("[CTRL] Rerun viewer launched")

    # ── ZMQ PUB socket ─────────────────────────────────────
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.SNDHWM, 4)   # buffer at most 4 frames to prevent memory bloat
    publisher.setsockopt(zmq.LINGER, 0)   # drop unsent on exit
    publisher.bind(f"tcp://*:{ZMQ_PORT}")
    print(f"[CTRL] ZMQ PUB bound to tcp://*:{ZMQ_PORT}")

    # ── OpenCV display window ──────────────────────────────
    cv2.namedWindow("Handheld Scanner - Camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Handheld Scanner - Camera", cam_w, cam_h)

    # ── ZMQ "Slow Joiner" mitigation ───────────────────────
    # A freshly connected SUB may miss the first few messages
    # because ZMQ needs time to propagate subscriptions.
    # Mitigation: brief sleep + periodic re-publish of info.
    info_dict = _make_info_dict(robot, camera, lidar, lidar_width)
    info_bytes = json.dumps(info_dict, ensure_ascii=False).encode("utf-8")

    time_module.sleep(1.0)  # allow potential subscribers to connect
    publisher.send_multipart([b"info", info_bytes])
    print("[CTRL] Info published (initial)")

    # ── Simulation main loop ────────────────────────────────
    step_count = 0
    try:
        while robot.step(timestep) != -1:
            ts_us = int(robot.getTime() * 1_000_000)
            step_count += 1

            # ── Camera ──────────────────────────────────
            img = camera.getImage()
            if img is not None:
                cam_payload = _make_camera_msg(img, cam_w, cam_h, ts_us)
                publisher.send_multipart([b"camera", cam_payload],
                                         zmq.NOBLOCK)
                # NOBLOCK: drop frame if subscriber is slow,
                # rather than blocking the simulation.

                # ── OpenCV display ────────────────────────
                img_array = np.frombuffer(img, dtype=np.uint8)\
                                .reshape((cam_h, cam_w, 4))
                img_bgr = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)
                cv2.imshow("Handheld Scanner - Camera", img_bgr)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 'q' or ESC
                    print("[CTRL] User pressed 'q', stopping")
                    break

            # ── LiDAR ─────────────────────────────────────
            ranges = lidar.getRangeImage()
            if ranges is not None:
                lid_payload = _make_lidar_msg(ranges, lidar_layers,
                                              lidar_width, ts_us)
                publisher.send_multipart([b"lidar", lid_payload],
                                         zmq.NOBLOCK)

                # ── Rerun point cloud viz ───────────────────
                lid_fov = lidar.getFov()
                lid_vfov = lidar.getVerticalFov()
                points = _lidar_to_pointcloud(
                    ranges, lidar_layers, lidar_width, lid_fov, lid_vfov)
                rr.set_time("sim_frame", sequence=step_count)
                rr.log("world/lidar", rr.Points3D(
                    points,
                    colors=[0, 220, 60],   # green
                    radii=0.02,
                ))

            # ── IMU ──────────────────────────────────────
            accel = imu_accel.getValues()
            gyro  = imu_gyro.getValues()
            if accel is not None and gyro is not None:
                imu_payload = _make_imu_msg(accel, gyro, ts_us)
                publisher.send_multipart([b"imu", imu_payload],
                                         zmq.NOBLOCK)

            # ── Periodic info re-publish (for late joiners) ─
            if step_count % PUBLISH_INTERVAL_SEC == 0:
                try:
                    publisher.send_multipart([b"info", info_bytes],
                                             zmq.NOBLOCK)
                except zmq.ZMQError:
                    pass

    except KeyboardInterrupt:
        print("[CTRL] Stopped by user")
    except Exception as e:
        print(f"[CTRL] FATAL: {e}")
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        publisher.close(linger=0)
        context.term()
        print("[CTRL] ZMQ context terminated")


if __name__ == "__main__":
    main()
