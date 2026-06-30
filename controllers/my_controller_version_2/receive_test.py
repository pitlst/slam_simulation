import zmq
import json
import cv2
from loguru import logger

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5555")

# 订阅所有主题；也可单独订阅 b"lidar"、b"imu" 等
socket.setsockopt(zmq.SUBSCRIBE, b"")

logger.info("订阅端已连接，等待 Webots 数据...")

try:
    while True:
        topic, payload = socket.recv_multipart()
        data = json.loads(payload)
        ts = data["timestamp"]

        if topic == b"camera":
            img: cv2.typing.MatLike = data["data"]
            logger.debug(f"[camera] ts={ts:.3f} shape={img.shape}")
            cv2.imshow("Received", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                logger.info("用户要求退出")
                break

        elif topic == b"lidar":
            points = data["data"]
            logger.debug(f"[lidar]  ts={ts:.3f} points={len(points)} min={points.min():.2f} max={points.max():.2f}")

        elif topic == b"accel":
            logger.debug(f"[accel]  ts={ts:.3f} data={data['data']}")

        elif topic == b"gyro":
            logger.debug(f"[gyro]   ts={ts:.3f} data={data['data']}")

except KeyboardInterrupt:
    logger.warning("\n关闭订阅端")
finally:
    socket.close()
    context.term()
