import math
import time
import sys
sys.path.append("C:\\Users\\zhousong\\OneDrive\\桌面\\电机控制代码\\MotorControl\\MotorControl")
from MotorControl import VelControl

motor = VelControl('COM8', 921600)

motor.registerID(1)

motor.setMotorMode(1, 0)
motor.setMotorMode(1, 2)
time.sleep(1)

q = 0
v_ref = 1
vel = 1
servo_time = 0.02

dT_sum = 0
T_KP = 1.5   # 比例增益
T_KI = -0.001 # 积分增益
T_KD = 0.1   # 微分增益

previous_dT = 0  # 用于计算微分项

try:
    while True:
        motor.send(1, vel)

        feedback = motor.feedback(1)
        if feedback:
            print(f"{feedback[1]}")  # 打印反馈速度

            # 参数辨识
            Td = 0.465 * math.sin(feedback[0]) + 0.095
            dT = Td - feedback[2]
        else:
            dT = 0

        # 计算积分项
        dT_sum += dT
        T_Int = T_KI * dT_sum

        # 防止积分项过大
        if T_Int > 2:
            dT_sum = 2 / T_KI
            T_Int = 2
        elif T_Int < -2:
            dT_sum = -2 / T_KI
            T_Int = -2

        # 计算微分项
        dT_Diff = (dT - previous_dT) / servo_time
        previous_dT = dT  # 更新上一次误差

        T_Deriv = T_KD * dT_Diff

        # 更新速度参考，加入比例、积分和微分项
        vel = v_ref + T_KP * dT + T_Int + T_Deriv

        time.sleep(servo_time - 0.002)

except KeyboardInterrupt:
    print("killing")
finally:
    motor.stopSend()
    motor.setMotorMode(1, 1)
    motor.stopFeedback()

