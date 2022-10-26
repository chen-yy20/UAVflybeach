#!/usr/bin/env python
# coding=utf-8

import rospy
import signal
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String, Bool

class CrossDemo:
    def __init__(self, robot_name):
        signal.signal(signal.SIGINT, self.sigint_handler)
        self.robot_name = robot_name
        self.cmd_twist = Twist()
        self.pose = Pose()

        self.bridge_ = CvBridge()
        self.image_detect = None

        self.cmd_pub = rospy.Publisher(self.robot_name+'/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.robot_name+'/odom', Odometry, self.odom_callback)
        self.image_sub_ = rospy.Subscriber("/AKM_1/camera/rgb/image_raw", Image, self.imagesubCallback)
        self.detect_pub = rospy.Publisher(self.robot_name+'/detect_result', String, queue_size=10)
        self.Main()
        rospy.spin()
    
    def Main(self):
        detect_2 = None
        detect_4 = None
        detect_5 = None
        result = String()   # detect_result

        # ------------------------------- 1 point
        while self.pose.position.y < 3.7:
            #print("                               Theta: "+str(self.GetTheta()))
            self.CarMove(5, 0)
            rospy.sleep(0.1)
            
        self.CarMove(0, 0)

        # ------------------------------- turn left
        self.Turn(1, 7)

        # ------------------------------- 2 point
        while (self.pose.position.y < 6 and self.pose.position.x > 1.8):
            self.CarMove(5, 0)
            rospy.sleep(0.1)
        
        self .CarMove(0, 0)

        # ------------------------------- turn right
        self.Turn(-1, 18)

        # ------------------------------- detect=====================
        rospy.sleep(2)
        print("Start detecting......point 2")
        if self.detect_ball_yellow():
            detect_2 = '2y'
            
        elif self.detect_ball_red():
            detect_2 = '2r'
            
        elif self.detect_ball_blue():
            detect_2 = '2b'
            
        else:
            detect_2 = '2e'
        
        print(detect_2)
        result.data = detect_2
        self.detect_pub.publish(result)

        # ------------------------------- turn left
        self.Turn(1, 10)

        # ------------------------------- 3 point
        while self.pose.position.y < 12.3:
            self.CarMove(5, 0)
            rospy.sleep(0.1)
        
        self .CarMove(0, 0)
        
        # ------------------------------- turn right
        self.Turn(-1, 18)

        while self.pose.position.x < 3:
            self.CarMove(5, 0)
            rospy.sleep(0.1)
        
        self .CarMove(0, 0)

        self.Turn(-1, 6)

        while self.pose.position.y < 12.8:
            self.CarMove(-5, 0)
            rospy.sleep(0.1)
        self.CarMove(0, 0)
        # ------------------------------- detect=====================
        rospy.sleep(2)
        print("Start detecting......point 4")
        if self.detect_ball_yellow():
            detect_4 = '4y'
            
        elif self.detect_ball_red():
            detect_4 = '4r'
            
        elif self.detect_ball_blue():
            detect_4 = '4b'
            
        else:
            detect_4 = '4e'

        print(detect_4)
        result.data = detect_4
        self.detect_pub.publish(result)

        # ------------------------------- 4 point
        while self.pose.position.y < 13.3:
            self.CarMove(-5, 0)
            rospy.sleep(0.1)
        
        self .CarMove(0, 0)

        # ------------------------------- turn right
        self.TurnB(1, 17)

        while self.pose.position.x < 3.8:
            self.CarMove(-5, 0)
            rospy.sleep(0.1)
        
        self .CarMove(0, 0)

        # ------------------------------- detect=====================
        print("Start detecting......point 5")
        if self.detect_ball_yellow():
            detect_5 = '5y'
            
        elif self.detect_ball_red():
            detect_5 = '5r'
            
        elif self.detect_ball_blue():
            detect_5 = '5b'
            
        else:
            detect_5 = '5e'
        
        print(detect_5)
        result.data = detect_5
        self.detect_pub.publish(result)

        rospy.loginfo("Racecar reached")



    def Turn(self, DIR, CNT):
        cnt = 0
        while cnt < CNT:
            self.CarMove(3, DIR)
            rospy.sleep(0.05)
            cnt = cnt + 1
        self.CarMove(0, 0)
    

    def TurnB(self, DIR, CNT):
        cnt = 0
        while cnt < CNT:
            self.CarMove(-3, DIR)
            rospy.sleep(0.05)
            cnt = cnt + 1
        self.CarMove(0, 0)


    def GetTheta(self):
        PI = math.pi
        c = self.pose.orientation.w
        s = self.pose.orientation.z
        theta1 = 2 * math.acos(c)
        theta2 = 2 * math.asin(s)
        print("("+str(c)+" , " +str(s)+")")
        theta = 0
        if(c >= 0 and s >= 0):
            theta = (theta1 + theta2)/2
        elif(c < 0 and s >=0):
            theta = (theta1 + PI - theta2)/2
        elif(c < 0 and s < 0):
            theta = (2 * PI - theta1 - theta2)/2
        else:
            theta = (4 * PI - theta1 + theta2)/2

        # print("1: "+str(theta1))
        # print("2: "+str(theta2))
        return theta


    def imagesubCallback(self, data):
        try:
            #将sensor_msgs/Image类型的消息转化为BGR格式图像
            self.image_detect = self.bridge_.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as err:
            print(err)

    def detect_ball(self):
        return (self.detect_ball_blue())or(self.detect_ball_red())or(self.detect_ball_())

    def detect_ball_yellow(self):
        color_range_ = [(26, 43, 46), (34, 255, 255)] # Yellow 的HSV范围 HSV颜色空间 H色调S饱和度V亮度 

        if self.image_detect is None:
            return False
        image_copy = self.image_detect.copy()
        height = image_copy.shape[0]
        width = image_copy.shape[1]

        frame = cv2.resize(image_copy, (width, height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道

        frame = cv2.inRange(frame, color_range_[0], color_range_[1])  # 对原图像和掩模进行位运算
        opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
        (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        # 在contours中找出最大轮廓
        contour_area_max = 0
        area_max_contour = None
        for c in contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                area_max_contour = c

        if area_max_contour is not None:
            if contour_area_max > 50:
                return True
        return False

    def detect_ball_red(self):
        color_range_ = [(3, 43, 46), (10, 255, 255)] # Red 的HSV范围 HSV颜色空间 H色调S饱和度V亮度 

        if self.image_detect is None:
            return False
        image_copy = self.image_detect.copy()
        height = image_copy.shape[0]
        width = image_copy.shape[1]

        frame = cv2.resize(image_copy, (width, height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道

        frame = cv2.inRange(frame, color_range_[0], color_range_[1])  # 对原图像和掩模进行位运算
        opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
        (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        # 在contours中找出最大轮廓
        contour_area_max = 0
        area_max_contour = None
        for c in contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                area_max_contour = c

        if area_max_contour is not None:
            if contour_area_max > 50:
                return True
        return False

    def detect_ball_blue(self):
        color_range_ = [(100, 43, 46), (124, 255, 255)] # Blue 的HSV范围 HSV颜色空间 H色调S饱和度V亮度 

        if self.image_detect is None:
            return False
        image_copy = self.image_detect.copy()
        height = image_copy.shape[0]
        width = image_copy.shape[1]

        frame = cv2.resize(image_copy, (width, height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道

        frame = cv2.inRange(frame, color_range_[0], color_range_[1])  # 对原图像和掩模进行位运算
        opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
        (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        # 在contours中找出最大轮廓
        contour_area_max = 0
        area_max_contour = None
        for c in contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                area_max_contour = c

        if area_max_contour is not None:
            if contour_area_max > 50:
                return True
        return False
    #--------------------------------------

    def odom_callback(self, msg):
        self.pose = msg.pose.pose


    def CarMove(self, x, z):
        self.cmd_twist.linear.x = x
        self.cmd_twist.angular.z = z
        self.cmd_pub.publish(self.cmd_twist)


    def sigint_handler(self, signum, frame):
        self.CarMove(0, 0)
        rospy.logwarn("Catched interrupt signal! Stop and exit...")
        exit()

if __name__ == '__main__':
    rospy.init_node("cross_demo_node")
    robot_name = rospy.get_param('~robot_name', 'AKM_1')
    try:
        CrossDemo(robot_name)
    except Exception as e:
        rospy.logerr(e)
    