import zmq
import cv2
import struct
import traceback
import zstandard
import time
import numpy as np
from loguru import logger

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5555")

# 订阅所有主题；也可单独订阅 b"lidar"、b"imu" 等
socket.setsockopt(zmq.SUBSCRIBE, b"")
# 设置接收超时 100ms，避免永久阻塞在 recv_multipart
socket.setsockopt(zmq.RCVTIMEO, 100)

decompressor = zstandard.ZstdDecompressor()

OPENCV_TITLE = "Camera Frame"
cv2.namedWindow(OPENCV_TITLE, cv2.WINDOW_NORMAL)

FPS_STATS = {b"camera": 0, b"lidar": 0, b"accel": 0, b"gyro": 0}
FPS_LAST_PRINT_TIME = time.time()

logger.info("订阅端已连接，等待 Webots 数据...")

try:
    while True:
        # 打印帧率
        now = time.time()
        if now - FPS_LAST_PRINT_TIME >= 1.0:
            elapsed = now - FPS_LAST_PRINT_TIME
            cam = FPS_STATS.get(b"camera", 0) / elapsed
            lid = FPS_STATS.get(b"lidar", 0) / elapsed
            acc = FPS_STATS.get(b"accel", 0) / elapsed
            gyr = FPS_STATS.get(b"gyro", 0) / elapsed
            logger.info(
                f"[Recv FPS] camera={cam:.1f} | lidar={lid:.1f} | accel={acc:.1f} | gyro={gyr:.1f}"
            )
            # 重置计数器
            FPS_STATS = {b"camera": 0, b"lidar": 0, b"accel": 0, b"gyro": 0}
            FPS_LAST_PRINT_TIME = now
        
        
        try:
            topic, payload = socket.recv_multipart()
        except zmq.Again:
            # 100ms 内没收到数据，继续循环检查键盘
            pass
        else:
            # logger.debug("收到消息")

            if topic == b"camera":
                FPS_STATS[b"camera"] += 1
                
                header_fmt = "<dIII"
                header_size = struct.calcsize(header_fmt)
                timestamp, w, h, c = struct.unpack_from(header_fmt, payload, 0)

                len_fmt = "<II"
                compressed_len, raw_len = struct.unpack_from(len_fmt, payload, header_size)
                data_offset = header_size + struct.calcsize(len_fmt)

                compressed = payload[data_offset:]
                raw_bytes = decompressor.decompress(compressed)
                img = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((h, w, c))

                # logger.debug(f"[camera] ts={timestamp:.3f} shape={img.shape}")
                cv2.imshow(OPENCV_TITLE, img)

            elif topic == b"lidar":
                FPS_STATS[b"lidar"] += 1
                timestamp, n_points = struct.unpack_from("<dI", payload, 0)
                points = np.frombuffer(payload[12:], dtype=np.float32)
                # logger.debug(f"[lidar]  ts={timestamp:.3f} points={len(points)} min={points.min():.2f} max={points.max():.2f}")

            elif topic == b"accel":
                FPS_STATS[b"accel"] += 1
                timestamp, x, y, z = struct.unpack("<dddd", payload)
                # logger.debug(f"[accel]  ts={timestamp:.3f} data=[{x:.4f}, {y:.4f}, {z:.4f}]")

            elif topic == b"gyro":
                FPS_STATS[b"gyro"] += 1
                timestamp, x, y, z = struct.unpack("<dddd", payload)
                # logger.debug(f"[gyro]   ts={timestamp:.3f} data=[{x:.4f}, {y:.4f}, {z:.4f}]")

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            logger.info("用户要求退出")
            break

except KeyboardInterrupt:
    logger.warning("\n关闭订阅端")
except Exception as e:
    traceback.print_exc()
finally:
    cv2.destroyAllWindows()
    socket.close()
    context.term()
