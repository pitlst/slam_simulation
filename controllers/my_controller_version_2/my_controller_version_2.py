import cv2
import numpy as np
import traceback
from loguru import logger
from controller import Robot


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    logger.debug(f"控制器初始化完成，当前时间戳为={timestep}ms")

    camera = robot.getCamera("MV-CS016-10UC-V5")
    lidar = robot.getLidar("LivoxMid360")
    imu_accel = robot.getAccelerometer("imu_accelerometer")
    imu_gyro = robot.getGyro("imu_gyro")
    if not camera or not lidar or not imu_accel or not imu_gyro:
        logger.error("获取不到对应的传感器设备，退出")
        return
    logger.debug("设备获取完成")

    camera.enable(timestep)
    lidar.enable(timestep)
    imu_accel.enable(timestep)
    imu_gyro.enable(timestep)
    if robot.step(timestep) == -1:
        logger.warning("仿真已停止，退出")
        return

    title = "Camera Frame"
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, camera.getWidth(), camera.getHeight())

    try:
        while robot.step(timestep) != -1:
            frame = camera.getImage()
            if frame is not None:
                frame_array = np.frombuffer(frame, dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
                frame_bgr = cv2.cvtColor(frame_array, cv2.COLOR_BGRA2BGR)
                cv2.imshow(title, frame_bgr)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    logger.info("用户要求退出")
                    break
            ranges = lidar.getRangeImage()
            if ranges is not None:
                logger.debug(f"ranges 是 {ranges}")
            accel = imu_accel.getValues()
            if accel is not None:
                logger.debug(f"accel 是 {accel}")
            gyro  = imu_gyro.getValues()
            if gyro is not None:
                logger.debug(f"gyro 是 {gyro}")
    except KeyboardInterrupt:
        logger.info("用户要求退出")
    except Exception as e:
        logger.error(f"FATAL: {e}")
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
