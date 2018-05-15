#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import time
import tf as t
import serial
import binascii
import threading
import roslib
import math
from std_msgs.msg import Float32 ,Float32MultiArray,Header,Int16MultiArray,Int8
from sensor_msgs.msg import Imu
from robot_imu.msg import EulerHeader
from geometry_msgs.msg import Twist,TwistStamped
# 从名为tl70d.py的文件中调用TL70D的类
from tl70d import TL70D

class Velocity_Fix(object):

    def __init__(self):

        rospy.init_node("velocity_fix",anonymous=True)

        self.nodename = rospy.get_name()

        rospy.loginfo("-I- %s started" % self.nodename)

        # 获得底盘两轮之间的宽度，单位是m
        self.w = rospy.get_param("~base_width", 0.37)

        # 直线前进时左右轮的速度控制命令初始化
        self.velLeftwheel = 0

        self.velRightwheel = 0

        # 修正后左右轮的速度控制命令初始化
        self._velLeftwheel = 0

        self._velRightwheel = 0

        # 陀螺仪方位角的参考值,每次均清零
        self.yaw_value_init = 0

        # 陀螺仪方位角的当前值
        self.yaw_value_now = 0

        # 陀螺仪方位角的上一个值
        self.yaw_value_last = 0

        # 当前方位角的测量时间
        self.time_value_now = 0

        self.time_inseconds_now = 0

        # 上一个方位角的测量时间
        self.time_value_last = 0

        self.time_inseconds_last = 0

        # 当前机器人控制命令的角速度和线速度
        self.linear_speed = 0

        self.angular_speed = 0

        # 速度修正更新频率
        self.rate = 10.0

        self.d_time = 1 / self.rate

        # 速度修正的周期频率 1Hz
        self.r = rospy.Rate(self.rate)

        # 计数
        self.count = 0

        self.yaw_theory = 0

        # 机器人使用陀螺仪修正过的直线前进命令话题，已经转化为电机控制命令
        self.line_pub = rospy.Publisher('line_speed_topic', Int16MultiArray, queue_size=100)

        # 机器人使用陀螺仪修正过的原地旋转命令话题，已经转化为电机控制命令
        self.rotate_pub = rospy.Publisher('rotate_speed_topic',Int16MultiArray, queue_size=100)

        self._fixed_speed = rospy.Publisher('fixed_speed_topic', Int16MultiArray, queue_size=100)

        self.reset_pub = rospy.Publisher('reset_topic',Int8,queue_size=10)

        # 订阅线速度话题
        self.linear_sub = rospy.Subscriber('linear_velocity_topic',Twist,self._Linear_Vel_Callback)

        # 订阅角速度话题
        self.angular_sub = rospy.Subscriber('angular_velocity_topic', Twist, self._Angular_Vel_Callback)

        # 订阅修正过后的速度话题消息
        self.velocity_sub = rospy.Subscriber('velocity_fixed_topic',TwistStamped,self._Velocity_Callback)

        # 订阅TL740D陀螺转角仪的方位角话题，单位是°
        self.yaw_sub = rospy.Subscriber("tl740d_euler_topic",EulerHeader,self._Tl740d_Euler_Callback)

        # 速度命令修正方位角阈值 单位是°
        self.yaw_theta = 1

        # 存储收到速度命令的时间点
        self.velocity_time_last = 0

        self.velocity_time = 0

        self.stack = [0,0]

        self.stack_time = [0,0]

        self.delt_time_ = 0

    def twistTomotor(self,linear_speed,angular_speed):

        self.right = 1.0 * linear_speed + angular_speed * self.w / 2

        self.left = 1.0 * linear_speed - angular_speed * self.w / 2

        return [self.right,self.left]

    def _Velocity_Callback(self,msg):

        self.linear_speed = msg.twist.linear.x

        self.angular_speed = msg.twist.angular.z

        # 左右轮速度分解,单位是rad/s
        [self.velRightwheel, self.velLeftwheel] = self.twistTomotor(self.linear_speed, self.angular_speed)

        if self.linear_speed != 0 or self.angular_speed != 0:

            # 堆栈stack的【0】元素表示上一次接收到速度命令的时间点,单位是秒
            self.stack[0] = self.stack[1]

            self.stack[1] = msg.header.stamp.secs + msg.header.stamp.nsecs / 1000000000.0

            # print self.stack[1] - self.stack[0]

            # 速度命令发布频率不小于10Hz
            self.delt_time = self.stack[1] - self.stack[0]

            if (self.delt_time - 0.5) > 0:

                self.msg = Int8()

                self.yaw_theory = 0

                self.msg.data = 1

                self.reset_pub.publish(self.msg)

                print  self.delt_time

    def _Tl740d_Euler_Callback(self,msg):

        # 单位是度
        self.yaw_value_now = msg.yaw

        # 单位是秒
        self.time_value_now = msg.stamp.secs + msg.stamp.nsecs / 1000000000.00

        self.stack_time[0] = self.stack_time[1]

        self.stack_time[1] = self.time_value_now

        # 计算收到两次imu数据的间隙
        self.delt_time_ = self.stack_time[1] - self.stack_time[0]

        if self.delt_time_ - 0.5 > 0 and self.stack_time[0] != 0:

            print self.delt_time_

    # 速度修正主程序
    def _Update(self):

        # 确保接收到下一时刻的方位角
        if  self.stack_time[1] > self.stack_time[0] and self.stack_time[0] != 0 and self.stack_time[1] != 0:

            # 当连续收到两次imu数据后才开始进行运动控制
            if self.delt_time_ - 0.5 < 0 :

                # 方位角理论值
                self.yaw_theory = self.yaw_theory + self.d_time * self.angular_speed * 180 / 3.1415926
                # self.yaw_theory = 0

                # rospy.loginfo("yaw_threory is %f   %f   %f",self.yaw_theory,self.angular_speed,self.d_time)

                # 判断方位角超出理论值时进行调整
                if abs(self.yaw_value_now - self.yaw_theory) - self.yaw_theta > 0:

                    if self.yaw_value_now - self.yaw_theory > 0 :

                        self.add_angular = -0.05

                    else:

                        self.add_angular = 0.05

                    # 需要附加的角速度的大小
                    [velRight, velLeft] = self.twistTomotor(0, self.add_angular)

                    self._velLeftwheel = self.velLeftwheel + velLeft

                    self._velRightwheel = self.velRightwheel + velRight

                    self.cmd_left_speed = self._velLeftwheel * 100

                    self.cmd_right_speed = self._velRightwheel * 100

                    # rospy.loginfo("调整过后的速度： %i %i %f", self.cmd_left_speed, self.cmd_right_speed, self.yaw_value_now)

                    self.msg_speed = Int16MultiArray()

                    self.msg_speed.data.append(self.cmd_left_speed)

                    self.msg_speed.data.append(self.cmd_right_speed)

                    if self.cmd_right_speed != 0 or self.cmd_left_speed != 0:

                        self._fixed_speed.publish(self.msg_speed)

                else:

                    self.cmd_right_speed = self.velRightwheel * 100

                    self.cmd_left_speed = self.velLeftwheel * 100

                    # rospy.loginfo("摆动没超过阈值： %i %i %f",self.cmd_left_speed,self.cmd_right_speed,self.yaw_value_now)

                    self.msg_speed = Int16MultiArray()

                    self.msg_speed.data.append(self.cmd_left_speed)

                    self.msg_speed.data.append(self.cmd_right_speed)

                    if self.cmd_right_speed != 0 or self.cmd_left_speed != 0:

                        self._fixed_speed.publish(self.msg_speed)

            else:

                self.cmd_left_speed = self.cmd_right_speed = 0

                # rospy.loginfo("速度置为0 ： %i %i", self.cmd_left_speed, self.cmd_right_speed)

                self.msg_speed = Int16MultiArray()

                self.msg_speed.data.append(self.cmd_left_speed)

                self.msg_speed.data.append(self.cmd_right_speed)

                if self.cmd_right_speed != 0 or self.cmd_left_speed != 0:

                    self._fixed_speed.publish(self.msg_speed)

        else:

            self.cmd_left_speed = self.cmd_right_speed = 0

            # rospy.loginfo("速度置为0 ： %i %i", self.cmd_left_speed, self.cmd_right_speed)

            self.msg_speed = Int16MultiArray()

            self.msg_speed.data.append(self.cmd_left_speed)

            self.msg_speed.data.append(self.cmd_right_speed)

            if self.cmd_right_speed != 0 or self.cmd_left_speed != 0:

                self._fixed_speed.publish(self.msg_speed)

    # 主循环
    def spin(self):

        while not rospy.is_shutdown():

            self._Update()

            self.r.sleep()

if __name__ == '__main__':

    try:

        vf = Velocity_Fix()

        vf.spin()

    except rospy.ROSInterruptException:

        rospy.logwarn("Error in main function")
