import sensor, image, time
from pyb import LED
import openmv_numpy as np
from kalman_filter import Tracker,Tracker_Manager
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_exposure(False)
#sensor.set_auto_exposure(False,100)

xm_list=[]
ym_list=[]
xp_list=[]
yp_list=[]


yellow_threshold=(24, 100, -28, 127, 19, 127)

A = np.array([[1,0,1,0],
              [0,1,0,1],
              [0,0,1,0],
              [0,0,0,1]])


H_k = np.eye(4)

Q = np.eye(4,value=0.1)

R = np.eye(4)

B=None

Manager = Tracker_Manager()

while(True):


    img = sensor.snapshot()#.lens_corr(1.8)

    #img.binary([yellow_rect])#二值化
    #img.dilate(1)
    #img.erode(1)


    blobs= img.find_blobs([yellow_threshold],x_stride=10,x_stride=10)
    s,x1,y1,w1,h1,position_x,position_y=0,0,0,0,0,0,0
    for i in blobs:#此方法为在嵌有字典的列表中索引值，i为每一个字典
        if i[2]*i[3] > s: #用类似冒泡排序一样找出色块最大面积
            s = i[2]*i[3]
            x1=i[0]
            y1=i[1]
            w1=i[2]
            h1=i[3]
            position_x=int(i[5])
            position_y=int(i[6])
    img.draw_rectangle((x1,y1,w1,h1),color=(255,0,0))#画框语句位于while循环中，做到每帧画一次
    xm_list.append(position_x)
    ym_list.append(position_y)
        #img.draw_rectangle(r.rect(), color = (255, 0, 0))

        #for p in r.corners(): img.draw_circle(p[0], p[1], 5, color = (0, 255, 0))

        #print(position_x,position_y)
    img.draw_cross(position_x, position_y, color = (255, 0, 0), size = 10, thickness = 2)
        #匹配
    Manager.match(position_x,position_y,A,H_k,Q,R)
    Manager.update()
    trails_pre = Manager.get_motion_trail_pre()
    for ID,trail in trails_pre:
        if len(trail):
            x,y = trail[0][0], trail[0][1]
            xp_list.append(x)
            yp_list.append(y)

            img.draw_cross(x, y, color = (255, 255, 0), size = 20, thickness = 2)


