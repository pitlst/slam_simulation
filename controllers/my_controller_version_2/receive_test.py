import zmq
import cv2
import struct
import traceback
import zlib
import numpy as np
from loguru import logger

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5555")

# 订阅所有主题；也可单独订阅 b"lidar"、b"imu" 等
socket.setsockopt(zmq.SUBSCRIBE, b"")
# 设置接收超时 100ms，避免永久阻塞在 recv_multipart
socket.setsockopt(zmq.RCVTIMEO, 100)

OPENCV_TITLE = "Camera Frame"
cv2.namedWindow(OPENCV_TITLE, cv2.WINDOW_NORMAL)

logger.info("订阅端已连接，等待 Webots 数据...")

try:
    while True:
        try:
            topic, payload = socket.recv_multipart()
        except zmq.Again:
            # 100ms 内没收到数据，继续循环检查键盘
            pass
        else:
            logger.debug("收到消息")

            if topic == b"camera":
                header_fmt = "<dIII"
                header_size = struct.calcsize(header_fmt)
                timestamp, w, h, c = struct.unpack_from(header_fmt, payload, 0)

                len_fmt = "<II"
                compressed_len, raw_len = struct.unpack_from(len_fmt, payload, header_size)
                data_offset = header_size + struct.calcsize(len_fmt)

                compressed = payload[data_offset:]
                raw_bytes = zlib.decompress(compressed)
                img = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((h, w, c))

                logger.debug(f"[camera] ts={timestamp:.3f} shape={img.shape}")
                cv2.imshow(OPENCV_TITLE, img)

            elif topic == b"lidar":
                timestamp, n_points = struct.unpack_from("<dI", payload, 0)
                points = np.frombuffer(payload[12:], dtype=np.float32)
                logger.debug(f"[lidar]  ts={timestamp:.3f} points={len(points)} min={points.min():.2f} max={points.max():.2f}")

            elif topic == b"accel":
                timestamp, x, y, z = struct.unpack("<dddd", payload)
                logger.debug(f"[accel]  ts={timestamp:.3f} data=[{x:.4f}, {y:.4f}, {z:.4f}]")

            elif topic == b"gyro":
                timestamp, x, y, z = struct.unpack("<dddd", payload)
                logger.debug(f"[gyro]   ts={timestamp:.3f} data=[{x:.4f}, {y:.4f}, {z:.4f}]")

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
