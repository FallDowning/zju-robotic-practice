import math
import time

import sys
sys.path.append("C:\\Users\\zhousong\\OneDrive\\桌面\\电机控制代码\\MotorControl\\MotorControl")

from MotorControl import VelControl

MAX_VEL = 5

# 设置串口号与波特率连接电机
motor = VelControl('COM8', 921600)

# 注册电机ID
motor.registerID(1)

# 电机（ID=1）使能
motor.setMotorMode(1, 0)

# 将当前位置设为零位
motor.setMotorMode(1, 2)
time.sleep(1)

# 设置速度为1rad/s
vel = 1
# 设置伺服周期为0.02s（由于操作系统的原因，建议伺服周期不小于0.01s）
servo_time = 0.02

try:
    # 在每个伺服周期中向电机（ID=1）发送指令，并得到电机的反馈数据
    while 1:
        # 判断速度是否大于最大速度
        if abs(vel) > MAX_VEL:
            vel = MAX_VEL if vel > 0 else -MAX_VEL
        # 向电机（ID=1）发送速度指令,
        motor.send(1, vel)

        # 接收电机（ID=1）的反馈数据, [位置rad，速度rad/s，力矩N.m，MOS温度℃，电机线圈温度℃]]
        feedback = motor.feedback(1)
        if feedback:
            print(f"{feedback[1]}")
            pass

        # 保证伺服周期的相对准确性, 0.002为程序运行补偿时间(可根据实际情况调整)
        time.sleep(servo_time-0.002)

#### 下方代码不建议更改 #####
except KeyboardInterrupt:
    # Ctrl+C 结束控制
    print("killing")
finally:
    # 停止向电机的命令发送
    motor.stopSend()
    # 电机（ID=1）失能, 并结束反馈接收
    motor.setMotorMode(1, 1)
    motor.stopFeedback()
