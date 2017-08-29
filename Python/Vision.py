# -*- coding: utf-8 -*-

from __future__ import division
from collections import deque
import math
import numpy as np
import RPi.GPIO as GPIO
import time
import serial
import cv2

print
print('      NUEDC 2017      ')
print('- In Longer We Trust -')
print

# 画面设置常量
cap_width=320
cap_height=240

# 端口常量
alarm=37

# HSV空间下颜色阈值
yellowLower=np.array([26,43,46])
yellowUpper=np.array([34,255,255])

# 追踪绘图变量
mybuffer=16
pts=deque(maxlen=mybuffer) # pts = points
counter=0

# 时间变量
timeStamp1=time.time()
timeStamp2=time.time()
interval1=1/15
interval2=1/20

# 误差变量
error_x=0
error_y=0
error_x_i=0
error_y_i=0
error_x_buf=0
error_y_buf=0

# 串口通信变量
thr=0
pitch=1500
roll=1500
yaw=1500
thr_h=0
thr_l=0
pitch_l=0
pitch_h=0
roll_l=0
roll_h=0
yaw_l=0
yaw_h=0
sum_check=0

# PID变量
pitch_kp=1
pitch_ki=0
pitch_kd=0
roll_kp=1
roll_ki=0
roll_kd=0
yaw_kp=0
yaw_ki=0
yaw_kd=0

# 查找变量
findMethod=0
findMethod_disp=findMethod
cx=cap_width/2
cx_disp=cx
cy=cap_height/2
cy_disp=cy
rad=0
distance=0

# GPIO初始化
GPIO.setmode(GPIO.BOARD) # 设置引脚编号模式
GPIO.setwarnings(False) # 取消GPIO被占用的警告
GPIO.setup(alarm,GPIO.OUT) # 设置GPIO模式
GPIO.output(alarm,1) # GPIO置高
print('GPIO OK!')

# 串口初始化
ser=serial.Serial("/dev/ttyAMA0",115200,timeout=1) # 阻塞限制1秒
print('Serial OK!')

# 摄像头初始化
# OpenCV
cap=cv2.VideoCapture(0) #连接到硬件摄像头
ret=cap.set(3,cap_width) #采集宽度
ret=cap.set(4,cap_height) #采集高度
# PiCamera
'''
camera=PiCamera() # 创建PiCamera对象
camera.resolution=(cap_width,cap_height) # 图像分辨率
camera.framerate=60 # 帧率
camera.iso=300 # 100和200是白天的合理值，而400和800对于低光更好
print('Camera adjusting...')
time.sleep(2)
print('Camera ISO: %d'%camera.iso)
camera.shutter_speed=camera.exposure_speed
# camera.shutter_speed=6500
camera.exposure_mode='off'
g=camera.awb_gains
# print(g)
# g=(3/1,6/5)
camera.awb_mode='off'
camera.awb_gains=g
rawCapture=PiRGBArray(camera,size=(cap_width,cap_height))
print('Camera exposure speed: %d' % camera.shutter_speed)
print('Camera white balance: %f, %f' % g)
'''
print('Camera OK!')

# 串口通信函数
def serialprint():
    global pitch_h,pitch_l,roll_h,roll_l,yaw_h,yaw_l
    global sum_check
    global timeStamp1,interval1
    if (time.time()>=timeStamp1+interval1):
        for i in range(3): # 连发3次数据
            # print('Times up! Send data.')
            # print('pitch=%d, roll=%d'%(pitch,roll))
            data=0xAC
            ser.write(chr(data)) # 帧头
            data=0x01
            ser.write(chr(data)) # 功能字
            ser.write(chr(pitch_h))
            ser.write(chr(pitch_l))
            ser.write(chr(roll_h))
            ser.write(chr(roll_l))
            ser.write(chr(yaw_h))
            ser.write(chr(yaw_l))
            sum_check=int((pitch_h+pitch_l+roll_h+roll_l+yaw_h+yaw_l)%256) # 求和校验位
            ser.write(chr(sum_check))
        timeStamp1=time.time() # 更新时间

# 单位转换函数
def data_trans():
    global pitch,roll,yaw,thr_l,thr_h,pitch_l,pitch_h,roll_l,roll_h,yaw_l,yaw_h
    pitch=int(pitch)
    roll=int(roll)
    yaw=int(yaw)
    pitch_h=pitch//256
    pitch_l=pitch%256
    roll_h=roll//256
    roll_l=roll%256
    yaw_h=yaw//256
    yaw_l=yaw%256

# 限幅函数1
def my_deadzone(x):
    if (x>2000):
        return 2000
    elif (x<1000):
        return 1000
    else:
        return x

# 限幅函数2
def data_anl():
    global error_x,error_y
    if (error_x<10 and error_x>0):
        error_x=0
    if (error_x>-10 and error_x<0):
        error_x=0
    if (error_y<10 and error_y>0):
        error_y=0
    if (error_y>-10 and error_y<0):
        error_y=0

# PID控制函数
def pid_control():
    global pitch,pitch_kp,pitch_ki,pitch_kd,error_x,error_x_buf,error_x_i
    global roll,roll_kp,roll_ki,roll_kd,error_y,error_y_buf,error_y_i
    global timeStamp2,interval2
    # pitch=my_deadzone((pitch_kp*error_x+pitch_ki*error_x_i+(error_x-error_x_buf)*pitch_kd)+1500)
    if (time.time()>=timeStamp2+interval2):
        pitch=my_deadzone(-(pitch_kp*error_y+pitch_ki*error_y_i+(error_y-error_y_buf)*pitch_kd)+1500)
        error_y_buf=error_y
        roll=my_deadzone(roll_kp*error_x+roll_ki*error_x_i+(error_x-error_x_buf)*roll_kd+1500)
        error_x_buf=error_x
        timeStamp2=time.time()

# 计算偏差函数
def get_error():
    global error_x,error_y,distance
    error_x=cx-cap_width/2
    error_y=cy-cap_height/2
    distance=math.sqrt(error_x*error_x+error_y*error_y)

# 检查轨迹
def check_diff():
    global cx,cx_disp,cy,cy_disp,findMethod,findMethod_disp
    if (cx_disp!=cx or cy_disp!=cy or findMethod_disp!=findMethod):
        print ('method=%d, x=%d, y=%d, distance=%d'%(findMethod,error_x,error_y,distance))
        cx_disp=cx
        cy_disp=cy
        findMethod_disp=findMethod

# 主函数部分
print('Initiate all done!')
print('Running... Press [Esc] to quit.')
# for frame in camera.capture_continuous(rawCapture,format="bgr", use_video_port=True): # PiCamera用，主循环
while(True):
    time1=time.time() # 时间戳1
    ret,frame=cap.read() # 捕获图像
    # image = frame.array # PiCamera用，捕获图像
    # 颜色特征追踪
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) # 转换颜色空间
    mask=cv2.inRange(hsv,yellowLower,yellowUpper) # 根据阈值构建掩膜
    mask=cv2.erode(mask,None,iterations=2) # 先腐蚀
    mask=cv2.dilate(mask,None,iterations=2) # 后膨胀
    cnts=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2] # 寻找轮廓
    center = None
    if (len(cnts)>0):
        findMethod=1
        c=max(cnts,key=cv2.contourArea) # 取面积最大的轮廓
        M=cv2.moments(c) # 计算轮廓的矩
        center=(int(M["m10"]/M["m00"]),int(M["m01"]/M["m00"])) # 计算质心坐标
        cx=center[0]
        cy=center[1]
        cv2.circle(frame,(cx,cy),3,(255,0,0),2)
    else:
        # Hough圆
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # 灰度化
        blurred=cv2.GaussianBlur(gray,(3,3),0) # 高斯模糊
        circles=cv2.HoughCircles(blurred,cv2.cv.CV_HOUGH_GRADIENT,1,100,param1=100,param2=35,minRadius=5,maxRadius=75)
        if circles is not None: # 至少有一个圆找到了
            findMethod=2
            circles=np.round(circles[0,:]).astype("int") # 转换所有坐标和半径为整形
            for (cx,cy,rad) in circles: # 遍历所有的圆
                cv2.circle(frame,(cx,cy),rad,(0,255,0),2)
        else:
            findMethod=0
            GPIO.output(alarm,1)
    cv2.imshow("Preview",frame) # 图像预览
    get_error()
    cm_xy=int(distance*3/14) # 像素缩放
    ultrawave=ser.read() # 从串口读一字节数据
    if (len(ultrawave)!=0):
        cm_z=int(str(ord(ultrawave)))
    else:
        cm_z=0
    ser.flushInput() # 清空缓冲区
    cm=math.sqrt(cm_xy*cm_xy+cm_z*cm_z) # 勾股定理算空间实际距离
    if (cm>=50 and cm<=150):
        GPIO.output(alarm,0) # GPIO电平拉低报警
    else:
        GPIO.output(alarm,1)
    data_anl()
    pid_control()
    data_trans()
    serialprint()
    check_diff()
    # rawCapture.truncate(0) # PiCamera用，清除图像流数据
    time2=time.time() # 时间戳2
    time_det=time2-time1 # 计算运行时长
    # print('fps: %.3f'%(1/time_det))
    key = cv2.waitKey(1)&0xFF # 等待按键
    if (key==27): # 按下[Esc]键退出
        break

# 清理工作
GPIO.cleanup(alarm) # 释放占用的GPIO
cap.release() # 释放摄像头
cv2.destroyAllWindows() # 关闭开启的一切窗口
print('Clean up everything.')
print('Program complete!')
