#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, SetBoolResponse

class LaserAvoidance:
    def __init__(self):
        rospy.init_node('laser_avoidance_node', anonymous=True)

        # 初始化参数
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)  # 最大线速度
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)  # 最大角速度
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.5)  # 障碍物检测阈值
        self.safety_distance = rospy.get_param('~safety_distance', 0.3)  # 安全距离
        self.detection_angle = rospy.get_param('~detection_angle', 90)  # 检测角度范围（度）

        # 当前状态
        self.laser_data = None
        self.is_active = False

        # 订阅话题
        rospy.Subscriber('scan', LaserScan, self.laser_callback)

        # 发布话题
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # 提供服务
        self.toggle_service = rospy.Service('toggle_avoidance', SetBool, self.toggle_callback)

        rospy.loginfo("Laser Avoidance Node Initialized")

    def laser_callback(self, msg):
        """处理激光雷达数据"""
        self.laser_data = msg

    def toggle_callback(self, req):
        """切换避障功能"""
        self.is_active = req.data
        if req.data:
            rospy.loginfo("Laser avoidance activated")
            return SetBoolResponse(success=True, message="Laser avoidance activated")
        else:
            rospy.loginfo("Laser avoidance deactivated")
            return SetBoolResponse(success=True, message="Laser avoidance deactivated")

    def find_best_direction(self):
        """找到最佳前进方向"""
        if self.laser_data is None:
            return 0.0, 0.0

        # 将激光雷达数据转换为直方图
        angle_increment = self.laser_data.angle_increment
        num_readings = len(self.laser_data.ranges)

        # 计算检测范围
        detection_rad = math.radians(self.detection_angle)
        start_angle = -detection_rad / 2
        end_angle = detection_rad / 2

        # 创建直方图
        histogram = np.zeros(num_readings)

        for i in range(num_readings):
            angle = self.laser_data.angle_min + i * angle_increment

            # 只考虑检测范围内的数据
            if start_angle <= angle <= end_angle:
                # 无效距离值设为0
                if self.laser_data.ranges[i] == float('inf') or math.isnan(self.laser_data.ranges[i]):
                    histogram[i] = 0
                else:
                    # 距离越远，权重越高
                    histogram[i] = self.laser_data.ranges[i]

        # 找到最佳方向
        best_index = np.argmax(histogram)
        best_angle = self.laser_data.angle_min + best_index * angle_increment
        best_distance = histogram[best_index]

        return best_angle, best_distance

    def calculate_velocity(self):
        """计算速度"""
        if not self.is_active or self.laser_data is None:
            # 停止移动
            cmd_vel = Twist()
            return cmd_vel

        # 检测前方障碍物
        front_clear = True
        front_threshold = self.obstacle_threshold

        # 检查前方区域
        center_index = len(self.laser_data.ranges) // 2
        front_range = int(math.radians(30) / self.laser_data.angle_increment)  # 前方30度

        for i in range(center_index - front_range, center_index + front_range):
            if 0 <= i < len(self.laser_data.ranges):
                if self.laser_data.ranges[i] < front_threshold:
                    front_clear = False
                    break

        cmd_vel = Twist()

        if front_clear:
            # 前方无障碍物，可以前进
            best_angle, best_distance = self.find_best_direction()

            # 计算线速度
            linear_speed = min(self.max_linear_speed, best_distance * 0.5)
            cmd_vel.linear.x = linear_speed

            # 计算角速度
            angular_speed = max(min(best_angle * 2.0, self.max_angular_speed), -self.max_angular_speed)
            cmd_vel.angular.z = angular_speed
        else:
            # 前方有障碍物，需要转向
            best_angle, _ = self.find_best_direction()

            # 只转向，不前进
            angular_speed = max(min(best_angle * 3.0, self.max_angular_speed), -self.max_angular_speed)
            cmd_vel.angular.z = angular_speed

        return cmd_vel

    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            cmd_vel = self.calculate_velocity()
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        avoidance = LaserAvoidance()
        avoidance.run()
    except rospy.ROSInterruptException:
        pass
