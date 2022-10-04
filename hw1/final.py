#!/usr/bin/env python
# coding=utf-8

import rospy
import signal
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import tf
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class MoveDemo:
    def __init__(self, robot_name):
        signal.signal(signal.SIGINT, self.sigint_handler)
        self.robot_name = robot_name
        self.cmd_twist = Twist()
        self.pose = Pose()
        self.cmd_pub = rospy.Publisher(self.robot_name+'/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.robot_name+'/odom', Odometry, self.odom_callback)
        self.image_sub_ = rospy.Subscriber("/AKM_1/camera/rgb/image_raw", Image, self.imagesubCallback)
        self.listener = tf.TransformListener()
        self.show = False
        self.move_procedure = [(0.5,0,11),(0.3,-0.6,3),(0.5,0,14),(0.3,0.6,3),(0.5,0,16),'show', 
        (0.3,0.6,3),(0.5,0,6),(0.3,-0.6,3),(0.5,0,10),(0.3,0.6,3),(0.5,0,4),(0.3,0.6,4),(0.5,0,2),'show',
        (0.3,-0.6,4),(0.5,0,8),(0.3,0.6,4),(0.5,0,18),(0.4,0.8,3),(0.5,0,5),'show',
        (0.4,-0.8,3),(0.3,-0.6,7),(0.5,0,17),(0.3,-0.6,3),(0.5,0,20),(0.3,0.6,3),(0.5,0,4)]
        self.Main()
        rospy.spin()
    
    def Main(self):
        # ============== moving procedure ==================
        for item in self.move_procedure:
            if item == 'show':
                self.show = True
            else:
                self.move(item[0],item[1],item[2])
        
    def move(self,x,z,time):
        for cnt in range(time):
            self.CarMove(x, z)
            rospy.sleep(0.2)
        self.CarMove(0, 0)
        rospy.loginfo("Racecar reached, stop!")

    def imagesubCallback(self,data):
        if self.show == True:
            try:
                bridge_ = CvBridge()
                #将sensor_msgs/Image类型的消息转化为BGR格式图像
                orgFrame_ = bridge_.imgmsg_to_cv2(data, 'bgr8')
                #打印图像尺寸
                orgFrame_copy = orgFrame_.copy()
                print("size of raw image: ",orgFrame_copy.shape)
            #在原始图像上画出矩形框
                # cv2.rectangle(orgFrame_copy, (100,100), (500,300), (255,0,0), 2)
            #将BGR图像再转换为sensor_msgs/Image消息格式发布
                cv2.imshow('ball!', orgFrame_copy)
                cv2.waitKey(0)
                self.takephoto = False

            except CvBridgeError as err:
                print(err)

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
        MoveDemo(robot_name)
    except Exception as e:
        rospy.logerr(e)
    