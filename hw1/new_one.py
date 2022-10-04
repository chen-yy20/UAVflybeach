#!/usr/bin/env python
# coding=utf-8
import rospy
import signal
import cv2
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from math import pi

class NavigateDemo:
    def __init__(self, robot_name):
        # sigint_handler 干扰信号
        signal.signal(signal.SIGINT, self.sigint_handler)
        self.robot_name = robot_name
        self.cmd_twist = Twist()
        self.pose = Pose()
        # 将twist 发布到机器人 cmd/vel的话题以控制机器人
        self.cmd_pub = rospy.Publisher(self.robot_name+'/cmd_vel', Twist, queue_size=1)
        # 订阅话题
        self.odom_sub = rospy.Subscriber(self.robot_name+'/odom', Odometry, self.odom_callback)
        self.stable_move()
        rospy.spin()
    
    def stable_move(self):
        while 1:
            self.CarMove(0,0.1*pi)
        while self.pose.position.y < 3:
            # print(self.pose.position.y)
            self.CarMove(1, 0)
            # rospy.sleep(0.5)
        self.CarMove(0, 0)
        rospy.loginfo("Racecar reached, stop!")

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
       # cmd_twist 
    def CarMove(self, x, z):
        self.cmd_twist.linear.x = x # linear.x 代表机器人正前方
        self.cmd_twist.angular.z = z # angular.z 代表旋转角
        # 发布这个话题，控制机器人运动
        self.cmd_pub.publish(self.cmd_twist)

    def sigint_handler(self, signum, frame):
        self.CarMove(0, 0)
        rospy.logwarn("Catched interrupt signal! Stop and exit...")
        exit()

    # ============== the CV part ==================
    def imagesubCallback(data):
        try:
            bridge_ = CvBridge()
            #将sensor_msgs/Image类型的消息转化为BGR格式图像
            orgFrame_ = bridge_.imgmsg_to_cv2(data, 'bgr8')
            #打印图像尺寸
            orgFrame_copy = orgFrame_.copy()
            print("size of raw image: ",orgFrame_copy.shape)
            #在原始图像上画出矩形框
            cv2.rectangle(orgFrame_copy, (100,100), (500,300), (255,0,0), 2)
            # 图像显示
            cv2.imshow('redpoint',orgFrame_copy)
            cv2.waitKey(0)

        except CvBridgeError as err:
            print(err)


if __name__ == '__main__':
    rospy.init_node("cross_demo_node")
    # 在AKM_1这个空间中获取机器人的name
    robot_name = rospy.get_param('~robot_name', 'AKM_1')
    try:
        NavigateDemo(robot_name) # 初始化一个机器人类
    except Exception as e:
        # 打印错误信息
        rospy.logerr(e)
    