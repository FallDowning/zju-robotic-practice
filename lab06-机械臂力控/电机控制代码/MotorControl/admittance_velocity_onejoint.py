import math
import time
import sys
sys.path.append("C:\\Users\\zhousong\\OneDrive\\桌面\\电机控制代码\\MotorControl\\MotorControl")

from MotorControl import VelControl

# 初始化电机控制
motor = VelControl('COM8', 921600)
motor.registerID(1)

# 设置电机模式
motor.setMotorMode(1, 0)  # 关闭电机模式
motor.setMotorMode(1, 2)  # 设置速度控制模式

# 初始参数
dq_d = 1  # 目标速度
dq_r = 1  # 参考速度
servo_time = 0.02  # 控制周期
M = 10  # 惯量
D = 80  # 阻尼系数

# PID控制器参数
Kp = 200# 比例增益
Ki = -0.05  # 积分增益
Kd = 60 # 微分增益

# 初始化 PID 控制状态
integral_T = 0  # 积分误差
previous_T_error = 0  # 上一次的力矩误差

motor.send(1, dq_r)  # 发送初始速度

try:
    while True:
        # 获取电机反馈
        feedback = motor.feedback(1)
        q = feedback[0]  # 位置反馈
        dq = feedback[1]  # 速度反馈
        T = 0.465 * math.sin(q - 0.1) - feedback[2]  # 计算力矩误差

        # 打印反馈速度
        print("feedback vel:", dq)

        # 计算 PID 控制输出
        T_error = T  # 假设目标力矩为0，因此力矩误差就是 T
        integral_T += T_error * servo_time  # 累计积分误差
        derivative_T = (T_error - previous_T_error) / servo_time  # 计算微分误差

        # 计算 PID 输出
        PID_output = Kp * T_error + Ki * integral_T + Kd * derivative_T
        previous_T_error = T_error  # 更新上一次误差

        # 使用 PID 输出调整速度变化率
        ddq_r = (- D * (dq_r - dq_d) + PID_output) / M
        dq_r = dq_r + ddq_r * servo_time  # 更新参考速度
        motor.send(1, dq_r)  # 发送新的参考速度
        time.sleep(servo_time - 0.002)  # 等待下一个控制周期

except KeyboardInterrupt:
    print("killing")
finally:
    motor.stopSend()
    motor.setMotorMode(1, 1)  # 设置为停止模式
    motor.stopFeedback()
