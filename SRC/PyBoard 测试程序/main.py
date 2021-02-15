#Copyright Deng（灯哥） (ream_d@yeah.net)  Py-apple dog project
#Github:https:#github.com/ToanTech/py-apple-quadruped-robot
#Licensed under the Apache License, Version 2.0 (the "License")
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at:http:#www.apache.org/licenses/LICENSE-2.0

from math import sin,cos,pi,sqrt,asin,acos
from pyb import UART
u1=UART(2,921600)   #Tx:x3 Rx：x4
u2=UART(1,921600)   #Tx:x9 Rx：x10
u3=UART(3,921600)   #Tx:y9 Rx：y10
u4=UART(4,921600)   #Tx:x1 Rx：x2

t=0
pi = 3.1415926;height=-100
x1=0;x2=0;x3=0;x4=0;Y1=0;y2=0;y3=0;y4=0
r1=1;r2=1;r3=1;r4=1
faai=0.5;Ts=1
xf=40;xs=-40;h=45
sita1_1=0;sita2_1=0
l1=35;l2=80


def trot():
    global Y1,y2,y3,y4
    global x1,x2,x3,x4
    if t<=Ts*faai:
        sigma=2*pi*t/(faai*Ts)
        zep=h*(1-cos(sigma))/2
        xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs
        xep_z=(xs-xf)*((sigma-sin(sigma))/(2*pi))+xf
        #输出y
        Y1=zep+height
        y2=0+height
        y3=zep+height
        y4=0+height
        #输出x
        x1=-xep_z*r1
        x2=-xep_b*r2
        x3=-xep_z*r3
        x4=-xep_b*r4
    elif t>Ts*faai and t<=Ts:
        sigma=2*pi*(t-Ts*faai)/(faai*Ts)
        zep=h*(1-cos(sigma))/2
        xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs
        xep_z=(xs-xf)*((sigma-sin(sigma))/(2*pi))+xf
        #输出y
        Y1=0+height
        y2=zep+height
        y3=0+height
        y4=zep+height
        #输出x
        x1=-xep_b*r1
        x2=-xep_z*r2
        x3=-xep_b*r3
        x4=-xep_z*r4

def ik():
    global sita1_1,sita2_1,Y1
    global sita1_2,sita2_2,y2
    global sita1_3,sita2_3,y3
    global sita1_4,sita2_4,y4
    #腿1
    Y1=-Y1
    L1=sqrt(x1*x1+Y1*Y1)
    psai1=asin(x1/L1)
    fai1=acos((L1*L1+l1*l1-l2*l2)/(2*l1*L1))
    sita1_1=round((fai1-psai1)*3,2)
    sita2_1=round((fai1+psai1)*3,2)
    #腿2
    y2=-y2
    L2=sqrt(x2*x2+y2*y2)
    psai2=asin(x2/L2)
    fai2=acos((L2*L2+l1*l1-l2*l2)/(2*l1*L2))
    sita1_2=round((fai2-psai2)*3,2)
    sita2_2=round((fai2+psai2)*3,2)
    #腿3
    y3=-y3
    L3=sqrt(x3*x3+y3*y3)
    psai3=asin(x3/L3)
    fai3=acos((L3*L3+l1*l1-l2*l2)/(2*l1*L3))
    sita1_3=round((fai3-psai3)*3,2)
    sita2_3=round((fai3+psai3)*3,2)
    #腿4
    y4=-y4
    L4=sqrt(x4*x4+y4*y4)
    psai4=asin(x4/L4)
    fai4=acos((L4*L4+l1*l1-l2*l2)/(2*l1*L4))
    sita1_4=round((fai4-psai4)*3,2)
    sita2_4=round((fai4+psai4)*3,2)
    
while True:
    t=t+0.01
    if t>=Ts:
        t=0
    trot()
    ik()
    u1.write(str(-sita1_1)+","+str(sita2_1)+'\n')
    u2.write(str(sita1_2)+","+str(-sita2_2)+'\n')
    u3.write(str(-sita1_4)+","+str(sita2_4)+'\n')
    u4.write(str(-sita1_3)+","+str(sita2_3)+'\n')
    #print("sita1_1",sita1_1)
    #print("sita2_1",sita2_1)
    print(str(sita1_2)+","+str(sita2_2))
