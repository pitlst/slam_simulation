import cv2
import zmq
import numpy as np
import time
import threading
import zlib
import queue
import traceback
from loguru import logger
from controller import Robot, Camera, Lidar, Accelerometer, Gyro
from dataclasses import dataclass

OPENCV_TITLE = "Camera Frame"
TIMESTEP = None

ZMQ_ADDRESS = "tcp://*:5555"    # 发布端地址
QUEUE_SIZE = 100                # 队列深度，防止订阅端消费慢时内存膨胀

DATA_QUEUE = queue.Queue(maxsize=QUEUE_SIZE)

VISUALIZATION = False  # 是否可视化


@dataclass
class sersor_object:
    robot: Robot
    camera: Camera
    lidar: Lidar
    imu_accel: Accelerometer
    imu_gyro: Gyro


def enqueue(topic: bytes, data: dict):
    '''非阻塞放入队列。队列满时自动丢弃最旧的一帧，保证仿真绝不卡顿'''
    payload = json.dumps(data)
    try:
        DATA_QUEUE.put_nowait((topic, payload))
    except queue.Full:
        # 队列满：丢弃最旧的一帧，再放入最新的（始终保持最新数据）
        try:
            DATA_QUEUE.get_nowait()
            DATA_QUEUE.put_nowait((topic, payload))
        except:
            pass


def sender_thread_func(socket: zmq.SyncSocket) -> None:
    while True:
        topic, payload = DATA_QUEUE.get()
        socket.send_multipart([topic, payload])


def main():
    global TIMESTEP
    # zmq init
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(ZMQ_ADDRESS)

    # robot init
    robot = Robot()
    TIMESTEP = int(robot.getBasicTimeStep())
    logger.debug(f"控制器初始化完成，当前时间戳为={TIMESTEP}ms")

    camera = robot.getCamera("MV-CS016-10UC-V5")
    lidar = robot.getLidar("LivoxMid360")
    imu_accel = robot.getAccelerometer("imu_accelerometer")
    imu_gyro = robot.getGyro("imu_gyro")
    if not camera or not lidar or not imu_accel or not imu_gyro:
        logger.error("获取不到对应的传感器设备，退出")
        return
    logger.debug("设备获取完成")

    camera.enable(TIMESTEP)
    lidar.enable(TIMESTEP)
    imu_accel.enable(TIMESTEP)
    imu_gyro.enable(TIMESTEP)
    if robot.step(TIMESTEP) == -1:
        logger.warning("仿真已停止，退出")
        return

    cam_h = camera.getHeight()
    cam_w = camera.getWidth()

    # opencv init
    if VISUALIZATION:
        cv2.namedWindow(OPENCV_TITLE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(OPENCV_TITLE, cam_w, cam_h)

    # 解决 PUB-SUB "慢连接" 问题：给订阅端留出 connect 时间
    logger.debug(f"发布端已绑定 {ZMQ_ADDRESS}，等待 1 秒让订阅端连接...")
    time.sleep(1)

    # 开始
    sender_thread = threading.Thread(target=sender_thread_func, args=(socket,), daemon=True)
    sender_thread.start()

    try:
        while robot.step(TIMESTEP) != -1:
            frame = camera.getImage()
            if frame is not None:
                frame_array = np.frombuffer(frame, dtype=np.uint8).reshape((cam_h, cam_w, 4))
                frame_bgr = cv2.cvtColor(frame_array, cv2.COLOR_BGRA2BGR)
                if VISUALIZATION:
                    cv2.imshow(OPENCV_TITLE, frame_bgr)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:
                        logger.info("用户要求退出")
                        break

                enqueue(b"camera", {
                    "timestamp": robot.getTime(),
                    "data": frame_bgr,
                    "encoding": "bgr8"
                })
            ranges = lidar.getRangeImage()
            if ranges is not None:
                if VISUALIZATION:
                    logger.debug(f"ranges 是 {ranges}")
                enqueue(b"lidar", {
                    "timestamp": robot.getTime(),
                    "data": np.array(ranges, dtype=np.float32),
                })

            accel = imu_accel.getValues()
            if accel is not None:
                if VISUALIZATION:
                    logger.debug(f"accel 是 {accel}")
                enqueue(b"accel", {
                    "timestamp": robot.getTime(),
                    "data": np.array(accel, dtype=np.float32),
                })

            gyro = imu_gyro.getValues()
            if gyro is not None:
                if VISUALIZATION:
                    logger.debug(f"gyro 是 {gyro}")
                enqueue(b"gyro", {
                    "timestamp": robot.getTime(),
                    "data": np.array(gyro, dtype=np.float32),
                })
    except KeyboardInterrupt:
        logger.info("用户要求退出")
    except Exception:
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        socket.close()
        context.term()


if __name__ == "__main__":
    main()
