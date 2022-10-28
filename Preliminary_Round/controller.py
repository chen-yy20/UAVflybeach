#!/usr/bin/python
#-*- encoding: utf8 -*-

# 对windows.world的一个简单控制策略
# 结合tello的控制接口，控制无人机从指定位置起飞，识别模拟火情标记（红色），穿过其下方对应的窗户，并在指定位置降落
# 本策略尽量使无人机的偏航角保持在初始值（90度）左右
# 运行roslaunch uav_sim windows.launch后，再在另一个终端中运行rostopic pub /tello/cmd_start std_msgs/Bool "data: 1"即可开始飞行
# 代码中的decision()函数和switchNavigatingState()函数共有3个空缺之处，需要同学们自行补全（每个空缺之处需要填上不超过3行代码）

from scipy.spatial.transform import Rotation as R
from collections import deque
from enum import Enum
import rospy
import cv2
import numpy as np
import math
from math import pi
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ControllerNode:
    class FlightState(Enum):  # 飞行状态
        WAITING = 1
        NAVIGATING = 2
        DETECTING_WINDOW = 3
        GOTO_BALL_1 = 4
        DETECTING_BALL_1 = 5
        GOTO_BALL_2 = 6
        DETECTING_BALL_2 = 7
        LANDING = 8

    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        rospy.logwarn('Controller node set up.')

        # 无人机在世界坐标系下的位姿
        self.R_wu_ = R.from_quat([0, 0, 0, 1])
        self.t_wu_ = np.zeros([3], dtype=np.float64)

        self.image_ = None
        self.color_range_red = [(0, 43, 46), (6, 255, 255)] # 红色的HSV范围 HSV颜色空间 H色调S饱和度V亮度 
        self.color_range_yellow = [(26, 43, 46), (34, 255, 255)] # 黄色
        self.color_range_blue = [(100, 43, 48), (124, 255, 255)] # 蓝色
        
        self.bridge_ = CvBridge()

        self.flight_state_ = self.FlightState.WAITING
        self.navigating_queue_ = deque()  # 存放多段导航信息的队列，队列元素为二元list，list的第一个元素代表导航维度（'x' or 'y' or 'z'），第二个元素代表导航目的地在该维度的坐标
        self.navigating_dimension_ = None  # 'x' or 'y' or 'z'
        self.navigating_destination_ = None
        self.next_state_ = None  # 完成多段导航后将切换的飞行状态
        self.result = ['e','e','e','e','e']

        self.window_x_list_ = [1.75, 4.25, 6.75] # 窗户中心点对应的x值

        self.is_begin_ = False

        self.window_index = 0

        self.commandPub_ = rospy.Publisher('/tello/cmd_string', String, queue_size=100)  # 发布tello格式控制信号
        # 无人机与无人车通信例程
        self.BallRcv_ = rospy.Subscriber('/detect_result',String,self.rcvCallback) # 接收来自小车的小球消息
        self.skip_1 = False

        self.poseSub_ = rospy.Subscriber('/tello/states', PoseStamped, self.poseCallback)  # 接收处理含噪无人机位姿信息
        self.imageSub_ = rospy.Subscriber('/iris/usb_cam/image_raw', Image, self.imageCallback)  # 接收摄像头图像
        self.imageSub_ = rospy.Subscriber('/tello/cmd_start', Bool, self.startcommandCallback)  # 接收开始飞行的命令

        self.resultPub_ = rospy.Publisher('/target_result', String, queue_size=100)



        # ================ 航点数组 ================
        self.navigate_queue_1 = [['z',3.5],['x',1.5],['r',90],['y',8.0],['z',1.75],['r',0]]
        # self.navigate_queue_2_new = [['z',3.5],['r',90],['y',14],['x',6.0],['r',-110],['z',2]]
        # self.navigate_queue_2_new = [['z',3.5],['r',90],['y',14.5],['x',7.0],['z',3.0],['r',-120]]
        self.navigate_queue_2_new = [['z',3.5],['r',90],['y',14.5],['x',7.0],['r',-125],['m',[6.5,14,-120]],['z',2]]
        self.navigate_queue_2_origin = [['z',3.7],['r',90],['y',14],['x',6.3],['r',-110],['z',2]]


        rate = rospy.Rate(0.3)
        while not rospy.is_shutdown():
            if self.is_begin_:
                self.decision()
            rate.sleep()
        rospy.logwarn('UAV controller node shut down.')

    # 按照一定频率进行决策，并发布tello格式控制信号
    def decision(self):
        if self.flight_state_ == self.FlightState.WAITING:  # 起飞并飞至离墙体（y = 3.0m）适当距离的位置
            rospy.logwarn('State: WAITING')
            # the movement command format
            self.publishCommand('takeoff')
            # self.navigating_queue_ = deque([['z',1.65],['y',1.5],['x',1.75]]) # 飞到第一个航点
            self.navigating_queue_ = deque([['z',1.65],['m',[1.75,1.5,90]]]) # 飞到第一个航点
            self.switchNavigatingState()
            self.next_state_ = self.FlightState.DETECTING_WINDOW

        elif self.flight_state_ == self.FlightState.NAVIGATING:
            rospy.logwarn('State: NAVIGATING')
            # 如果yaw与90度相差超过正负10度，需要进行旋转调整yaw
            # 先不要管四元数的原理了，只需要用这一行转换为欧拉角，yaw对应偏航，pitch对应俯仰，roll对应翻滚，我们只需要调整偏航
            # 飞机头指向y轴，为90°
           

            if self.navigating_dimension_ == 'x':
                dim_index = 0
            elif self.navigating_dimension_== 'y':
                dim_index = 1
            elif self.navigating_dimension_== 'z':
                dim_index = 2
            elif self.navigating_dimension_== 'r': # 设置r为角度的改变
                dim_index = 3
            elif self.navigating_dimension_== 'm': # 设置m为位置微调
                dim_index = 4



            # 也是看目标点与当前位置在对应维度下的坐标差异
            if dim_index < 3:
                print("CURRENT POSITION:",self.t_wu_)
                dist = self.navigating_destination_ - self.t_wu_[dim_index]
                if abs(dist) < 0.2:  # 当前段导航结束
                    self.switchNavigatingState()
                else:
                    dir_index = 0 if dist > 0 else 1  # direction index
                    # 根据维度（dim_index）和导航方向（dir_index）决定使用哪个命令
                    command_matrix = [['right ', 'left '], ['forward ', 'back '], ['up ', 'down ']]
                    command = command_matrix[dim_index][dir_index]
                    if abs(dist) > 4:
                        self.publishCommand(command+'300')
                    else:
                        self.publishCommand(command+str(int(abs(100*dist))))
            elif dim_index == 3:
                (yaw, pitch, roll) = self.R_wu_.as_euler('zyx', degrees=True)
                angle = self.navigating_destination_
                print("Euler:",str(yaw),str(pitch),str(roll))
                yaw = yaw+360 if yaw <0 else yaw
                angle = angle+360 if angle<0 else angle
                yaw_diff = yaw - angle
                if abs(yaw_diff) < 5 or abs(yaw_diff-360)<5:  # 当前段导航结束
                    self.switchNavigatingState()
                else:
                    if yaw_diff  > 0:  # clockwise
                        self.publishCommand('cw %d' % (int(yaw_diff)))
                        # if abs(yaw_diff)>40:
                        #     rospy.sleep(1) # wait 1s
                        #     self.publishCommand('back %d' % (50)) # 调整旋转导致的位姿偏移
                        return
                    elif yaw_diff < 0:  # counterclockwise
                        self.publishCommand('ccw %d' % (int(-yaw_diff)))
                        # if abs(yaw_diff)>40:
                        #     rospy.sleep(1)
                        #     self.publishCommand('back %d' % (50)) # 调整旋转导致的位姿偏移
                        return
            elif dim_index == 4:
                (yaw, _, _) = self.R_wu_.as_euler('zyx', degrees=True)
                if self.micro_move(self.t_wu_[0],self.t_wu_[1],yaw,self.navigating_destination_[0],self.navigating_destination_[1],self.navigating_destination_[2]):
                    self.switchNavigatingState()


    

        elif self.flight_state_ == self.FlightState.DETECTING_WINDOW:
            rospy.logwarn('State: DETECTING_WINDOW')
            # TODO: 此处不同的目标有不同的表示高度和角度，需要在此阶段进行调整
            # 如果无人机飞行高度与标识高度（1.75m）相差太多，则需要进行调整
            if self.t_wu_[2] > 2.0:
                self.publishCommand('down %d' % int(100*(self.t_wu_[2] - 1.75)))
                return
            elif self.t_wu_[2] < 1.5:
                self.publishCommand('up %d' % int(-100*(self.t_wu_[2] - 1.75)))
                return
            # 如果yaw与90度相差超过正负10度，需要进行旋转调整yaw
            (yaw, pitch, roll) = self.R_wu_.as_euler('zyx', degrees=True)
            yaw_diff = yaw - 90 if yaw > -90 else yaw + 270
            if yaw_diff > 10:  # clockwise
                self.publishCommand('cw %d' % (int(yaw_diff) if yaw_diff > 15 else 15))
                return
            elif yaw_diff < -10:  # counterclockwise
                self.publishCommand('ccw %d' % (int(-yaw_diff) if yaw_diff < -15 else 15))
                return

            #  TODO：向裁判机特定话题发送检测结果
            if self.detectTarget():
                # 根据无人机当前x坐标判断正确的窗口是哪一个
                # win_dist = [abs(self.t_wu_[0]-win_x) for win_x in self.window_x_list_]
                # win_index = win_dist.index(min(win_dist))  # 正确的窗户编号
                print('Window detected:',str(self.window_index))
                # 移动到窗户前方0.6m，降低飞行高度为1m，移动到对应窗户的正中心，前进穿过窗户到y=4m，移动到降落位置x=7m
                # 穿过窗户以后，移动到第一个观察点的位置，见arena.png
                self.navigating_queue_ = deque([['z', 1.2], ['x', self.window_x_list_[self.window_index]], ['r',90],['y', 4.0]])  # 通过窗户并导航至终点上方
                self.switchNavigatingState()
                self.next_state_ = self.FlightState.GOTO_BALL_1
            else:
                if self.t_wu_[0] > 7.5:
                    rospy.loginfo('Detection failed, ready to land.')
                    self.flight_state_ = self.FlightState.LANDING
                else:  # 向右侧平移一段距离，继续检测
                #     self.publishCommand('right 75')
                    print(self.window_x_list_,self.t_wu_)
                    dis = abs(self.window_x_list_[self.window_index+1]-self.t_wu_[0])
                    print("窗户{}没有着火。右移{}米".format(self.window_index,dis))
                    self.window_index += 1
                   
                    self.publishCommand('right %d' % (int(100*dis)-20))

        elif self.flight_state_ == self.FlightState.GOTO_BALL_1:
            if not self.skip_1:
                rospy.logwarn('State: GOTO_BALL_1')
                self.navigating_queue_ = deque(self.navigate_queue_1)
                self.switchNavigatingState()
                self.next_state_ = self.FlightState.DETECTING_BALL_1
            else:
                self.flight_state_ = self.FlightState.GOTO_BALL_2

        elif self.flight_state_ == self.FlightState.DETECTING_BALL_1:
            rospy.logwarn('State: DETECTING_BALL_1')
            self.detect_ball_1()
            self.next_state_ = self.FlightState.GOTO_BALL_2
            self.switchNavigatingState()
            
        
        elif self.flight_state_ == self.FlightState.GOTO_BALL_2:
            rospy.logwarn('State: GOTO_BALL_2')
            if self.skip_1:
                self.navigating_queue_ = deque(self.navigate_queue_2_new)
            else:
                self.navigating_queue_ = deque(self.navigate_queue_2_origin)
            self.switchNavigatingState()
            self.next_state_ = self.FlightState.DETECTING_BALL_2

        elif self.flight_state_ == self.FlightState.DETECTING_BALL_2:
            rospy.logwarn('State: DETECTING_BALL_2')
            if self.skip_1:
                rospy.sleep(3)
                #self.publishCommand('right 90')
            self.detect_ball_2()
            self.next_state_ = self.FlightState.LANDING
            self.switchNavigatingState()


        elif self.flight_state_ == self.FlightState.LANDING:
            rospy.logwarn('State: LANDING')
            #self.publishCommand('back 100')
            #rospy.sleep(5)
            self.publishCommand('land')

            # 在landing阶段向无人车发布自己观察到的结果
            Result = String()
            self.result = ''.join(self.result)
            print('Result:', self.result)
            Result.data = self.result # 注意需要以这种方式打包一下result

            self.resultPub_.publish(Result)
        else:
            pass

    # 在向目标点导航过程中，更新导航状态和信息
    def switchNavigatingState(self):
        if len(self.navigating_queue_) == 0:
            self.flight_state_ = self.next_state_
        else: # 从队列头部取出无人机下一次导航的状态信息
 
            next_nav = self.navigating_queue_.popleft()
            print("next_nav:",next_nav)
            self.navigating_dimension_ = next_nav[0]
            self.navigating_destination_ = next_nav[1] # 更新新的目标航点，一次只更新一个维
            self.flight_state_ = self.FlightState.NAVIGATING # 切换到巡航状态

    # 位置微调
    def micro_move(self,x,y,theta,target_x,target_y,target_yaw):
        print("START MICRO MOVE")
        target_x = target_x -x
        target_y = target_y -y
        l = math.sqrt(target_x*target_x+target_y*target_y)
        print("斜边和目标角度sin:")
        print(l,target_y/l)
        phi = math.asin(target_y/l)
        print("phi_sin:",math.sin(phi))
        if target_x <0:
            phi = pi-phi
        print("phi:",str(phi/pi)+"*pi")
        # 转为弧度制
        new_theta = theta/180.0*pi
        delta_theta = new_theta-phi
        print(delta_theta)
        front = l*math.cos(delta_theta)
        front_arrive = False
        heng_arrive = False

        command = ""
        if front >0.2:
            command = "forward "+str(int(100*front))
            self.publishCommand(command)
        elif front <-0.2:
            command = "back "+str(int(100*-front))
            self.publishCommand(command)
        else:
            print("NO FRONT MOVE, BIAS:",str(front))
            front_arrive = True
        rospy.sleep(2)
        command = ""
        heng = l*math.sin(delta_theta)
        if heng >0.2:
            command = "right "+str(int(100*heng))
            self.publishCommand(command)
        elif heng <-0.2:
            command = "left "+str(int(100*-heng))
            self.publishCommand(command)
        else:
            print("NO HENG MOVE, BIAS:",str(heng))
            heng_arrive = True
        if front_arrive and heng_arrive:
            print("YAW DETECTING!")
            rospy.sleep(2)
            theta = 360+theta if theta<0 else theta
            target_yaw = 360+target_yaw if target_yaw<0 else target_yaw
            yaw_diff = target_yaw - theta
            print("YAW_DIFF:",yaw_diff)
            if abs(yaw_diff)<5:
                print("CORRECT YAW DIFF:",yaw_diff)
                return True
            elif yaw_diff>0:
                command = "ccw " +str(int(yaw_diff))
                self.publishCommand(command)
            elif yaw_diff<0:
                command = "cw " +str(int(-yaw_diff))
                self.publishCommand(command)

            
        return False


    
    # 判断是否检测到目标
    def detectTarget(self):
        print("开始检测目标")
        if self.image_ is None:
            return False
        image_copy = self.image_.copy()
        height = image_copy.shape[0]
        width = image_copy.shape[1]

        frame = cv2.resize(image_copy, (width, height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道
        color_range_red_for_fire = [(0, 43, 46), (6, 255, 255)]
        frame = cv2.inRange(frame, color_range_red_for_fire[0], color_range_red_for_fire[1])  # 对原图像和掩模进行位运算
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

    # TODO： 检测小球的颜色函数
    def detect_ball_1(self):
        print("到达检测点1，开始检测")
        if self.image_ is None:
            return
        image_copy = self.image_.copy()
        
        height = image_copy.shape[0]
        width = image_copy.shape[1]
        frame = cv2.resize(image_copy, (width, height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道

        print('开始检测是否为红球')
        frame_red = cv2.inRange(frame, self.color_range_red[0], self.color_range_red[1])  # 对原图像和掩模进行位运算
        dilated_red = cv2.dilate(frame_red, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2) # 膨胀
        circles_red = cv2.HoughCircles(dilated_red, cv2.HOUGH_GRADIENT, 1, 100, param1=15, param2=7, minRadius=8, maxRadius=100)
        print(circles_red)
        if circles_red is not None:
            print('detected red')
            self.result[1] = 'r'
            return
        
        print('开始检测是否为黄球')
        frame_yellow = cv2.inRange(frame, self.color_range_yellow[0], self.color_range_yellow[1])  # 对原图像和掩模进行位运算
        dilated_yellow = cv2.dilate(frame_yellow, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2) # 膨胀
        circles_yellow = cv2.HoughCircles(dilated_yellow, cv2.HOUGH_GRADIENT, 1, 100, param1=15, param2=7, minRadius=8, maxRadius=100)
        print(circles_yellow)
        if circles_yellow is not None:
            print('detected yellow')
            self.result[1] = 'y'
            return

        print('开始检测是否为蓝球')
        frame_blue = cv2.inRange(frame, self.color_range_blue[0], self.color_range_blue[1])  # 对原图像和掩模进行位运算
        dilated_blue = cv2.dilate(frame_blue, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2) # 膨胀
        circles_blue = cv2.HoughCircles(dilated_blue, cv2.HOUGH_GRADIENT, 1, 100, param1=15, param2=7, minRadius=8, maxRadius=100)
        print(circles_blue)
        if circles_blue is not None:
            print('detected blue')
            self.result[1] = 'b'
            return
        print('detected empty')
        return

    # location 1.3.4
    def detect_ball_2(self):
        print("到达监测点2，开始检测")
        
        if self.image_ is None:
            return
        image_copy = self.image_.copy()
        cv2.imshow('检测点2',image_copy)
        cv2.waitKey(0)
        height = image_copy.shape[0]
        width = image_copy.shape[1]
        frame = cv2.resize(image_copy, (width, height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道

        print(frame.shape)

        print('开始检测是否为红球')
        frame_red = cv2.inRange(frame, self.color_range_red[0], self.color_range_red[1])  # 对原图像和掩模进行位运算
        dilated_red = cv2.dilate(frame_red, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2) # 膨胀
        circles_red = cv2.HoughCircles(dilated_red, cv2.HOUGH_GRADIENT, 1, 100, param1=15, param2=7, minRadius=5, maxRadius=100)
        print(circles_red)
        if circles_red is not None:
            print('detected red')
            if circles_red[0][0][0] < 100 and self.result[0] == 'e':
                self.result[0] = 'r'
            elif circles_red[0][0][0] >= 100 and circles_red[0][0][0] < 200 and self.result[2] == 'e':
                self.result[2] = 'r'
            elif circles_red[0][0][0] >= 200 and self.result[3] == 'e':
                self.result[3] = 'r'
        
        print('开始检测是否为黄球')
        frame_yellow = cv2.inRange(frame, self.color_range_yellow[0], self.color_range_yellow[1])  # 对原图像和掩模进行位运算
        dilated_yellow = cv2.dilate(frame_yellow, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2) # 膨胀
        circles_yellow = cv2.HoughCircles(dilated_yellow, cv2.HOUGH_GRADIENT, 1, 100, param1=15, param2=7, minRadius=5, maxRadius=100)
        print(circles_yellow)
        if circles_yellow is not None:
            print('detected yellow')
            if circles_yellow[0][0][0] < 100 and self.result[0] == 'e':
                self.result[0] = 'y'
            elif circles_yellow[0][0][0] >= 100 and circles_yellow[0][0][0] < 200 and self.result[2] == 'e':
                self.result[2] = 'y'
            elif circles_yellow[0][0][0] >= 200 and self.result[3] == 'e':
                self.result[3] = 'y'

        print('开始检测是否为蓝球')
        frame_blue = cv2.inRange(frame, self.color_range_blue[0], self.color_range_blue[1])  # 对原图像和掩模进行位运算
        dilated_blue = cv2.dilate(frame_blue, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2) # 膨胀
        circles_blue = cv2.HoughCircles(dilated_blue, cv2.HOUGH_GRADIENT, 1, 100, param1=15, param2=7, minRadius=5, maxRadius=100)
        print(circles_blue)
        if circles_blue is not None:
            print('detected blue')
            if circles_blue[0][0][0] < 100 and self.result[0] == 'e':
                self.result[0] = 'b'
            elif circles_blue[0][0][0] >= 100 and circles_blue[0][0][0] < 200 and self.result[2] == 'e':
                self.result[2] = 'b'
            elif circles_blue[0][0][0] >= 200 and self.result[3] == 'e':
                self.result[3] = 'b'
        


    # 向相关topic发布tello命令
    def publishCommand(self, command_str):
        print("发布指令：",command_str)
        msg = String()
        msg.data = command_str
        self.commandPub_.publish(msg)

    # 接收无人机位姿
    def poseCallback(self, msg):
        self.t_wu_ = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.R_wu_ = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        pass

    # 接收相机图像
    def imageCallback(self, msg):
        try:
            self.image_ = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as err:
            print(err)

    def rcvCallback(self,msg):
        print("收到小车信息：")
        data = msg.data
        print(data)
        self.result[int(data[0])-1] = data[1]
        # 2号点非空，更改航线
        if data[0] == '2' and data[1] != 'e':
            print("2号点非空，更改航线")
            self.skip_1 = True



    # 接收开始信号
    def startcommandCallback(self, msg):
        self.is_begin_ = msg.data


if __name__ == '__main__':
    cn = ControllerNode()

