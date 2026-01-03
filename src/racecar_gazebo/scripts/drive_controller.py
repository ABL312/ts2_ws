#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class DriveController:
    def __init__(self):
        rospy.init_node('drive_controller', anonymous=True)

        # 初始化参数
        self.wheel_base = rospy.get_param('~wheel_base', 0.335)  # 轴距
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.073)  # 轮胎半径
        self.max_steering_angle = rospy.get_param('~max_steering_angle', 0.5)  # 最大转向角

        # 当前速度指令
        self.current_cmd = Twist()

        # 订阅话题
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)

        # 发布话题
        self.left_rear_wheel_pub = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=10)
        self.right_rear_wheel_pub = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=10)
        self.left_front_wheel_pub = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=10)
        self.right_front_wheel_pub = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=10)
        self.left_steering_pub = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=10)
        self.right_steering_pub = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=10)

        rospy.loginfo("Drive Controller Node Initialized")

    def cmd_callback(self, msg):
        """处理速度指令"""
        self.current_cmd = msg

    def calculate_wheel_speeds(self):
        """计算轮子速度"""
        # 获取线速度和角速度
        linear_vel = self.current_cmd.linear.x
        angular_vel = self.current_cmd.angular.z

        # 计算左右轮速度
        left_wheel_speed = (linear_vel - angular_vel * self.wheel_base / 2) / self.wheel_radius
        right_wheel_speed = (linear_vel + angular_vel * self.wheel_base / 2) / self.wheel_radius

        # 限制轮速
        max_wheel_speed = 35.0  # 最大轮速 (rad/s)
        left_wheel_speed = max(min(left_wheel_speed, max_wheel_speed), -max_wheel_speed)
        right_wheel_speed = max(min(right_wheel_speed, max_wheel_speed), -max_wheel_speed)

        return left_wheel_speed, right_wheel_speed

    def calculate_steering_angles(self):
        """计算转向角度 - 使用阿克曼转向几何"""
        linear_vel = self.current_cmd.linear.x
        angular_vel = self.current_cmd.angular.z

        # 避免除以零
        if abs(linear_vel) < 0.01:
            return 0.0

        # 正确公式: delta = atan(L * omega / v)
        steering_angle = math.atan(self.wheel_base * angular_vel / linear_vel)

        # 限制转向角度
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)

        return steering_angle

    def publish_commands(self):
        """发布控制指令"""
        # 计算轮速
        left_speed, right_speed = self.calculate_wheel_speeds()

        # 计算转向角
        steering_angle = self.calculate_steering_angles()

        # 发布轮速指令
        self.left_rear_wheel_pub.publish(Float64(left_speed))
        self.right_rear_wheel_pub.publish(Float64(right_speed))
        self.left_front_wheel_pub.publish(Float64(left_speed))
        self.right_front_wheel_pub.publish(Float64(right_speed))

        # 发布转向指令
        self.left_steering_pub.publish(Float64(steering_angle))
        self.right_steering_pub.publish(Float64(steering_angle))

    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            self.publish_commands()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = DriveController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
