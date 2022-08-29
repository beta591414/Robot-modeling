from Robot import Robot
import numpy as np
import until
import time
import pandas as pd
import math
import serial

def test():
    # 打开设备管理器确定连接的COM口，linux和mac只要选择对应串口就行，需要根据具体的串口进行更改，但是波特率不要改
    r = Robot(com='COM6', baud=250000)
    # 连接到真实机器人
    r.connect()
    # 控制周期（推荐周期，最好不要改）
    T = 0.02
    # 初始化时间
    t = 0
    # 对应三点的关节值
    qO = np.array([0, 0, 0, 0,  0, 0]) / math.pi * 180
    qA0 = np.array([-0.73841174, 0.73410972, 1.65482929, -0.84614813, -0.03075535, -0.06804614]) / math.pi * 180
    qA =  np.array([-0.69841174, 0.73410972, 1.56082929, -0.86614813, -0.03075535, -0.06804614])/math.pi*180
    qB =  np.array([-0.08845569, 0.11414149, 1.54840006, -0.5902079, 0.04097188,-0.07763637])/math.pi*180
    #qC = np.array([0.73568646, 0.27145575, 1.61017211, -0.1854593, -0.10481398, 1.51148932]) / math.pi * 180
    qC =  np.array([0.92568646, 0.29145575, 1.61017211, -0.2054593 , -0.10481398,  1.51148932])/math.pi*180
    #qC = np.array([0.85568646, 0.29145575, 1.61017211, -0.2354593, -0.10481398, 1.51148932]) / math.pi * 180
    # 规划代码可以写这里，提前规划好曲线，这里只是匀速规划曲线（效果肯定是不行的）
    # 规划的从零位到A点的时间，以2秒为例
    tOA = 1
    # 规划曲线为匀速曲线,仅仅用于从机械臂的零位到A点

    # A到B点时间
    tAB = 0.30
    # A到C点时间
    tAC = 0.84
    # A一个循环返回A的时间
    tAA = 1.50
    # tAB =0.32
    # # A到C点时间
    # tAC = 0.84
    # # A一个循环返回A的时间
    # tAA = 1.52
    # 过B点速度,单位是（度/秒）
    midVel = 15
    v1 = (qA - 0) / tOA
    # 规划A点到A点经过B,C点的曲线，quinticCurvePlanning3见源代码
    k0 = until.quinticCurvePlanning(qO, qA, tOA)
    k = until.quinticCurvePlanning3(qA,qB,qC,qA, tAA, tAB,tAC)
    # 规划完成

    #开始控制机械臂运动
    # 使用该函数可以使机械臂回到零位
    r.go_home()
    # 开始控制机器人
    # 原点到A点
    n=0
    while(1):
        start = time.time()
        # 重新开始一次循环

        if t >= tOA + tAA:
            n = n + 1
            t = tOA
            print('Turn finished : ',n,end='\n')
        # 通过时间与规划目标关节位置的关系，得到挡墙时刻下期望机械臂关节到达的位置
        if t < tOA:
            q = until.quinticCurveExcute(k0,t)
        elif t >= tOA and t <tOA + tAA:
            q = until.quinticCurveExcute3(k,t-tOA)

        # 控制机械臂运动，syncMove输入格式为1*6的np.array，单位为度，代表的含义是当前周期下机械臂关节的位置
        r.syncMove(q)
        # 更新时间
        t = t + T
        # 定时器操作
        end = time.time()
        spend_time = end - start
        if spend_time < T:
            time.sleep(T - spend_time)
        else:
            print("timeout!")


if __name__ == '__main__': 
    test()
  