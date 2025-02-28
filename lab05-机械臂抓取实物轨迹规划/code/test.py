from Robot.Robot import Robot
import numpy as np
import until
import time
import pandas as pd
import math
from myIK import IKSolver


def test():
    q_array = []
    r = Robot(com='COM5', baud=250000)
    r.connect()
    T = 0.02
    t = 0

    iks = IKSolver()

    qA_ = np.array([0.37, -0.09, 0.165, np.pi, 0, -np.pi/2])
    qA = iks.solve(qA_)[:, 2]
    print(qA)
    qB_ = np.array([0.288, -0.288, 0.165, np.pi, 0, -np.pi/2])
    qB = iks.solve(qB_)[:, 2]

    # q0 = np.array([-32.4, 26.7, 52.38, 1.75, -5.8, 1.14])
    q0 = np.zeros(6)

    # the first cube
    # q1 = np.array([-6.416, 76.2, 70.4, -54.756, 0.615, -0.615])
    q1_1 = np.array([ 0.348004121976367, -0.022031675538176,  0.09, -np.pi,0,-np.pi/2]) 
    q1 = iks.solve(q1_1)[:, 2]
    
    # the object pose 1,
    # q2 = np.array([-83.845, 82.62, 51.15, -41.3, 1.49, -0.43])
    q2_1 = np.array([   0.051780861973690, -0.389209243590751, 0.090117187219158, -178.49/180 * np.pi, 0, 10.78/180*np.pi])
    q2 = iks.solve(q2_1)[:, 2]

    # the second cube
    # q3 = np.array([12.832, 77.34, 65.127, -51.5, 0.96, -0.53])
    q3_1 = np.array([ 0.356112642250006,0.097050798735177, 0.093340875363385, -np.pi, 0, -np.pi/2])
    q3 = iks.solve(q3_1)[:, 2]

    # the object pose 2
    # q4 = np.array([-82.177, 77.16, 44.03, -22.32, -3.96, -0.52])
    q4_1 = np.array([  0.051780861973690, -0.389209243590751, 0.129990711054528,-178.49/180 * np.pi, 0, 10.78/180*np.pi])
    q4 = iks.solve(q4_1)[:, 2]

    q5 = np.array([-83.49609375, 63.01757812,
                   29.79492188, 0.96679688, 0, 0])

    q6 = np.array([7.82226562, 53.87695312, 60.46875, -
                  19.42382812, 12.83203125, 0.61523438])
    
    q7_1 = np.array([   0.051780861973690, -0.389209243590751, 0.215206767132024, -np.pi,0,0])
    q7 = iks.solve(q7_1)[:, 2]

    tO1 = 1.2
    t1A = 1.2
    tAB = 1.2
    tB2 = 1.2
    t72 = 1.2
    t2O = 1.8
    tO3 = 1.8
    t36 = 1.2
    t6A = 1.2
    tB5 = 1.2
    t54 = 1.2
    t_buff = 1

    vo1 = (q1-q0)/tO1
    # v1A = (qA-q1)/t1A
    # vAB = (qB-qA)/tAB
    # vB2 = (q2-qB)/tB2

    vA = (np.array([-1.71663009e+01,  6.60437634e+01,  6.19368716e+01, -3.79806349e+01, 5.72957795e-15, -1.71663009e+01])
          - np.array([-1.71341349e+01,  6.60528610e+01, 6.19162960e+01, -3.79691570e+01, 5.72957795e-15, -1.71341349e+01]))/(tAB/1000)
    vB = (np.array([-4.82100285e+01, 7.47944383e+01, 4.25252492e+01, -2.73196876e+01, 5.72957795e-15, -4.82100285e+01])
          - np.array([-4.81828115e+01, 7.47610811e+01, 4.25980378e+01, -2.73591189e+01, 5.72957795e-15, -4.81828115e+01]))/(tAB/1000)

    k1A = until.quinticCurvePlanning(q1, qA, np.zeros(6), vA, t1A)
    kAB = until.splinePlanning(qA_[:3], qB_[:3])
    kB2 = until.quinticCurvePlanning(qB, q7, vB, np.zeros(6), tB2)
    k72 = until.quinticCurvePlanning(q7, q2, np.zeros(6), np.zeros(6), t72)
    k2O = until.quinticCurvePlanning(q2, q0, np.zeros(6), np.zeros(6), t2O)
    kO3 = until.quinticCurvePlanning(q0, q3, np.zeros(6), np.zeros(6), tO3)
    k36 = until.quinticCurvePlanning(q3, q6, np.zeros(6), np.zeros(6), t36)
    k6A = until.quinticCurvePlanning(q6, qA, np.zeros(6), vA, t6A)
    kB5 = until.quinticCurvePlanning(qB, q5, vB, np.zeros(6), tB5)
    k54 = until.quinticCurvePlanning(q5, q4, np.zeros(6), np.zeros(6), t54)
    # k = until.quinticCurvePlanning2(qA, qB, qC, midVel, tAB, tAC)

    t_tot = tO1 + t1A + tAB + tB2 + \
        t2O + tO3 + t_buff * 4 + t36 + t6A + tAB + tB5 + t54 + 0.1

    r.go_home()
    while (1):
        start = time.time()
        if t >= t_tot:
            print('Control Finished')
            r.go_home()
            # print(q_array)
            break
        # O->1
        if t < tO1:
            q = vo1*t
            print("... ")
        # 1->A
        elif t >= tO1 + t_buff and t < (tO1 + t_buff + t1A):
            q = until.quinticCurveExcute(k1A, t-tO1-t_buff)
            print("... ")
        # A->B
        elif t >= (tO1 + t_buff + t1A) and t < (tO1 + t_buff + t1A + tAB):
            q = until.splineExcute(kAB, t-tO1-t1A-t_buff, tAB)
            print("... ")
        # B->7
        elif t >= (tO1 + t_buff + t1A + tAB) and t < (tO1 + t_buff + t1A + tAB + tB2):
            q = until.quinticCurveExcute(kB2, t-tO1-t1A-tAB-t_buff)
        # 7->2
        elif t >= (tO1 + t_buff + t1A + tAB+ tB2) and t < (tO1 + t_buff + t1A + tAB + tB2 + t72):
            q = until.quinticCurveExcute(k72, t-tO1-t1A-tAB-tB2-t_buff)
        # 2->O
        elif t >= (tO1 + t_buff + t1A + tAB + tB2 + t_buff+ t72) and t < (tO1 + t_buff + t1A + tAB + tB2 + t_buff + t2O+ t72):
            q = until.quinticCurveExcute(k2O, t-tO1-t1A-tAB-tB2-2*t_buff-t72)
        # O->3
        elif t >= (tO1 + t_buff + t1A + tAB + tB2 + t_buff + t2O+ t72) and t < (tO1 + t_buff + t1A + tAB + tB2 + t_buff + t2O + tO3+ t72):
            q = until.quinticCurveExcute(kO3, t-tO1-t1A-tAB-tB2-t2O-2*t_buff-t72)
        # 3->6
        elif t >= (tO1 + t_buff + t1A + tAB + tB2 + t_buff + t2O + tO3 + t_buff+ t72) and t < (tO1 + t_buff + t1A + tAB + tB2 + t_buff + t2O + tO3 + t36 + t_buff+ t72):
            q = until.quinticCurveExcute(
                k36, t-tO1-t1A-tAB-tB2-t2O-tO3-3*t_buff-t72)
        # 6->A
        elif t >= (tO1 + t_buff + t1A + tAB + tB2 + t_buff + t2O + tO3 + t36 + t_buff+ t72) and t < (tO1 + t_buff + t1A + tAB + tB2 + t_buff + t2O + tO3 + t36 + t_buff + t6A+ t72):
            q = until.quinticCurveExcute(
                k6A, t-tO1-t1A-tAB-tB2-t2O-tO3-t36-3*t_buff-t72)
        # A->B
        elif t >= (tO1 + t_buff + t1A + tAB + tB2 + t_buff + t2O + tO3 + t36 + t_buff + t6A+ t72) and t < (tO1 + t_buff + t1A + tAB + tB2 + t_buff + t2O + tO3 + t36 + t_buff + t6A + tAB+ t72):
            q = until.splineExcute(
                kAB, t-tO1-t1A-tAB-tB2-t2O-tO3-t36-t6A-3*t_buff-t72, tAB)
        # B->5
        elif t >= (tO1 + t_buff + t1A + tAB + tB2 + t_buff + t2O + tO3 + t36 + t_buff + t6A + tAB+ t72) and t < (tO1 + t_buff + t1A + tAB + tB2 + t_buff + t2O + tO3 + t36 + t_buff + t6A + tAB + tB5+ t72):
            q = until.quinticCurveExcute(
                kB5, t-tO1-t1A-tAB-tB2-t2O-tO3-t36-t6A-tAB-3*t_buff-t72)
        # 5->4
        elif t >= (tO1 + t_buff + t1A + tAB + tB2 + t_buff + t2O + tO3 + t36 + t_buff + t6A + tAB + tB5+ t72) and t < (tO1 + t_buff + t1A + tAB + tB2 + t_buff + t2O + tO3 + t36 + t_buff + t6A + tAB + tB5 + t54 + t72):
            q = until.quinticCurveExcute(
                k54, t-tO1-t1A-tAB-tB2-t2O-tO3-t36-t6A-tAB-tB5-3*t_buff-t72)

        r.syncMove(np.reshape(q, (6, 1)))
        t = t + T

        end = time.time()
        spend_time = end - start
        if spend_time < T:
            time.sleep(T - spend_time)
        else:
            print("timeout!")

        q_array.append(r.syncFeedback())


if __name__ == '__main__':
    test()
