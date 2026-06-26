from loguru import logger
from controller import Robot

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    logger.debug(f"控制器初始化完成，当前时间戳为={timestep}ms")
    
    camera = robot.getCamera("MV-CS016-10UC-V5")
    lidar  = robot.getLidar("LivoxMid360")
    imu_accel = robot.getAccelerometer("imu_accelerometer")
    imu_gyro  = robot.getGyro("imu_gyro") 
    if not camera or not lidar or not imu_accel or not imu_gyro:
        logger.error("获取不到对应的传感器设备，退出")
        return
    logger.debug("设备获取完成")

    camera.enable(timestep)
    lidar.enable(timestep)
    imu_accel.enable(timestep)
    imu_gyro.enable(timestep)
    if robot.step(timestep) == -1:
        return


if __name__ == "__main__":
    main()
