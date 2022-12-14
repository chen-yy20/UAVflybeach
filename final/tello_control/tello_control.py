#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import threading
import random
import numpy as np
from enum import Enum
from collections import deque
import math
from math import pi

import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# if you can not find cv2 in your python, you can try this. usually happen when you use conda.
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import cv2
import tello_base as tello

y_max_th = 200
y_min_th = 170

img = None
tello_state='mid:-1;x:100;y:100;z:-170;mpry:1,180,1;pitch:0;roll:0;yaw:-19;'
tello_state_lock = threading.Lock()    
img_lock = threading.Lock()    

# send command to tello
class control_handler: 
    def __init__(self, control_pub):
        self.control_pub = control_pub
    
    def forward(self, cm):
        command = "forward "+(str(cm))
        self.control_pub.publish(command)
        print(command)
    
    def back(self, cm):
        command = "back "+(str(cm))
        self.control_pub.publish(command)
        print(command)
    
    def up(self, cm):
        command = "up "+(str(cm))
        self.control_pub.publish(command)
        print(command)
    
    def down(self, cm):
        command = "down "+(str(cm))
        self.control_pub.publish(command)
        print(command)

    def right(self, cm):
        command = "right "+(str(cm))
        self.control_pub.publish(command)
        print(command)

    def left(self, cm):
        command = "left "+(str(cm))
        self.control_pub.publish(command)
        print(command)

    def cw(self, cm):
        command = "cw "+(str(cm))
        self.control_pub.publish(command)
        print(command)

    def ccw(self, cm):
        command = "ccw "+(str(cm))
        self.control_pub.publish(command)
        print(command)

    def takeoff(self):
        command = "takeoff"
        self.control_pub.publish(command)
        print ("ready")
        
    def mon(self):
        command = "mon"
        self.control_pub.publish(command)
        print ("mon")

    def land(self):
        command = "land"
        self.control_pub.publish(command)

    def stop(self):
        command = "stop"
        self.control_pub.publish(command)

#subscribe tello_state and tello_image
class info_updater():   
    def __init__(self):
        rospy.Subscriber("tello_state", String, self.update_state)
        # ????????????????????????????????????
        rospy.Subscriber("tello_image", Image, self.update_img)
        self.con_thread = threading.Thread(target = rospy.spin)
        self.con_thread.start()

    def update_state(self,data):
        global tello_state, tello_state_lock
        tello_state_lock.acquire() #thread locker
        tello_state = data.data
        tello_state_lock.release()
        # print(tello_state)

    def update_img(self,data):
        global img, img_lock
        img_lock.acquire()#thread locker
        img = CvBridge().imgmsg_to_cv2(data, desired_encoding = "passthrough")
        img_lock.release()
        # print(img)


# put string into dict, easy to find
def parse_state():
    global tello_state, tello_state_lock
    tello_state_lock.acquire()
    statestr = tello_state.split(';')
    # print (statestr)
    dict={}
    for item in statestr:
        if 'mid:' in item:
            mid = int(item.split(':')[-1])
            dict['mid'] = mid
        elif 'x:' in item:
            x = int(item.split(':')[-1])
            dict['x'] = x
        elif 'z:' in item:
            z = int(item.split(':')[-1])
            dict['z'] = z
        elif 'mpry:' in item:
            mpry = item.split(':')[-1]
            mpry = mpry.split(',')
            dict['yaw'] = int(mpry[1])
        # y can be recognized as mpry, so put y first
        elif 'y:' in item:
            y = int(item.split(':')[-1])
            dict['y'] = y
    tello_state_lock.release()
    return dict

def showimg():
    global img, img_lock
    img_lock.acquire()
    cv2.imshow("tello_image", img)
    cv2.waitKey(2)
    img_lock.release()

# mini task: take off and fly to the center of the blanket.
class task_handle():
    class taskstages(Enum):
        finding_location  = 0 # find locating blanket 
        navigation = 1
        passing_door = 2
        order_location  = 3 # find the center of locating blanket and adjust tello 
        finished = 6 # task done signal

    def __init__(self , ctrl):
        self.States_Dict = None
        self.ctrl = ctrl
        self.now_stage = self.taskstages.finding_location
        self.navigation_queue = deque()

    def main(self): # main function: examine whether tello finish the task
        while not (self.now_stage == self.taskstages.finished):
            if(self.now_stage == self.taskstages.finding_location):
                self.finding_location()
            elif(self.now_stage == self.taskstages.order_location):
                self.order_location()
        self.ctrl.land()
        print("Task Done!")
    
    def finding_location(self): # find locating blanket (the higher, the easier)
        assert (self.now_stage == self.taskstages.finding_location)
        # ???????????????
        while not ( parse_state()['mid'] > 0 ): # if no locating blanket is found:
            distance = random.randint(20,30) # randomly select distance
            print (distance)
            # ????????????????????????
            if (self.States_Dict['z'] < 150):
                self.ctrl.up(distance) # tello up
            time.sleep(4) # wait for command finished
            # showimg()
        print("Find locating blanket!")
        self.now_stage = self.taskstages.order_location

    # target format: [x,y,z,yaw]
    def arrive_target(self,target):
        self.States_Dict = parse_state()
        print(self.States_Dict)
        if self.States_Dict['mid'] < 0 :
            self.now_stage = self.taskstages.finding_location
            print("----------- LOST LOCATION !!! -------------")
            return (False)
        delta_x = 0 if target[0]==404 else target[0]-self.States_Dict['x']
        delta_y = 0 if target[1]==404 else target[1]-self.States_Dict['y']
        delta_z = 0 if target[2]==404 else target[2]-self.States_Dict['z']
        delta_yaw = 0 if target[3]==404 else target[3]-self.States_Dict['yaw']
        result = False
        if (abs(delta_x)<15 and abs(delta_y)<15 and abs(delta_z)<15 and abs(delta_yaw)<8):
            result = True
        ans = (result,delta_x,delta_y,delta_z,delta_yaw)
        print(ans)
        return ans

    # target format: [x,y,z,yaw]
    # ?????? [x,y,z,yaw] ??????????????????404????????????????????????????????????????????????
    def target_move(self,target):
        sleep_time = 5
        while (1):
            delta = self.arrive_target(target) 
            if delta[0]:
                break

            if delta[3]>15:
                self.ctrl.up(delta[3])
                time.sleep(sleep_time)
            elif delta[3]<-15:
                self.ctrl.down(-delta[3])
                time.sleep(sleep_time)
           
            if delta[4] > 8:
                self.ctrl.ccw(delta[4])
                time.sleep(sleep_time)
            elif delta[4]<-8:
                self.ctrl.cw(-delta[4])
                time.sleep(sleep_time)

            if delta[1]>15:
                self.ctrl.right(delta[1])
                time.sleep(sleep_time)
            elif delta[1]<-15:
                self.ctrl.left(-delta[1])
                time.sleep(sleep_time)

            if delta[2]>15:
                self.ctrl.forward(delta[2])
                time.sleep(sleep_time)
            elif delta[2]<-15:
                self.ctrl.back(-delta[2])
                time.sleep(sleep_time)
        print("Arrive at :",target)

    def fly_upto(self,z):
        delta_z = 12
        while(abs(delta_z)>10):
            self.States_Dict = parse_state()
            delta_z = z-self.States_Dict['z']
            if delta_z>10:
                self.ctrl.up(int(delta_z))
            elif delta_z<-10:
                self.ctrl.down(int(-delta_z))
    
    def micro_move(self,target):
        print("START MICRO MOVE")
        while(1):
            delta = self.arrive_target(target)
            theta = target[3]-delta[3]
            if (delta[0]):
                break

            l = math.sqrt(delta[1]*delta[1]+delta[2]*delta[2])
            # print("?????????????????????sin:")
            # print(l,target_y/l)
            phi = math.asin(delta[2]/l)
            # print("phi_sin:",math.sin(phi))
            if delta[1] <0:
                phi = pi-phi
            # print("phi:",str(phi/pi)+"*pi")
            # ???????????????
            rad_theta = theta/180.0*pi
            delta_theta = phi-rad_theta
            print(delta_theta)
            front = l*math.cos(delta_theta)
            front_arrive = False
            heng_arrive = False

            if front > 15:
                ctrl.forward(int(front))
            elif front <-15:
                ctrl.back(int(-front))
            else:
                print("NO FRONT MOVE, BIAS:",str(front))
                front_arrive = True
            time.sleep(3)

            heng = l*math.sin(delta_theta)
            if heng > 15:
                ctrl.left(int(heng))
            elif heng <-15:
                ctrl.right(int(-heng))
            else:
                print("NO HENG MOVE, BIAS:",str(heng))
                heng_arrive = True
            

            if front_arrive and heng_arrive:
                print("YAW ADJUSTING!")
                time.sleep(3)
                theta = 360+theta if theta<0 else theta
                target_yaw = 360+target_yaw if target_yaw<0 else target_yaw
                yaw_diff = target_yaw - theta
                print("YAW_DIFF:",yaw_diff)
                if abs(yaw_diff)<8:
                    print("CORRECT YAW DIFF:",yaw_diff)
                    break

                elif yaw_diff>0:
                    ctrl.ccw(int(yaw_diff))
                elif yaw_diff<0:
                    ctrl.cw(int(-yaw_diff))

                

    # ?????????????????????
    def total_navigation(self):
        assert (self.now_stage == self.taskstages.finding_location)
        self.navigation_queue = deque([[]])

    def red_point_detect(self):
        # TODO: ????????????????????????????????????????????????
        pass

    # TODO: ????????????cv??????????????????????????????
    def passing_door(self):
        assert (self.now_stage == self.taskstages.passing_door)
        window_list = [] # format???[[x,-220,z,90]...]
        self.target_move([-45,-225,160,90]) #TODO: ?????????????????? 
        window_index = self.red_point_detect()
        self.target_move(window_list[window_index])
        ctrl.forward(60)
        print("Window passed!")
        self.now_stage = self.taskstages.navigation
        


    def order_location(self):# adjust tello to the center of locating blanket
        assert (self.now_stage == self.taskstages.order_location)
        state_conf = 0
        # ??????????????????
        self.States_Dict = parse_state()
        target = [-100,30,150,90]
        self.target_move(target)
        self.ctrl.stop()
        state_conf += 1
        print("stop")
        showimg()
        self.now_stage = self.taskstages.finished    


if __name__ == '__main__':
    rospy.init_node('tello_control', anonymous=True)

    control_pub = rospy.Publisher('command', String, queue_size=1)
    ctrl = control_handler(control_pub)
    infouper = info_updater()
    tasker = task_handle(ctrl)
    
    time.sleep(2)
    ctrl.mon()
    time.sleep(5)
    while(1):
        if parse_state()['mid'] == -1:
            ctrl.takeoff( )
            print("take off")
            break
    #print("mon")
    time.sleep(4)
    ctrl.up(60)
    print("up 60")
    time.sleep(2)

    tasker.main()

    ctrl.land()

    

