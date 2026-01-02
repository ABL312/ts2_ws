#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import tf
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse

class PointNavigation:
    def __init__(self):
        rospy.init_node('point_navigation_node', anonymous=True)

        # 初始化参数
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.1)  # 目标容差
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)  # 最大线速度
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)  # 最大角速度
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.5)  # 障碍物检测阈值

        # 当前状态
        self.current_pose = None
        self.goal_point = None
        self.is_navigating = False
        self.laser_data = None

        # 订阅话题
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.Subscriber('scan', LaserScan, self.laser_callback)

        # 发布话题
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # 提供服务
        self.set_goal_service = rospy.Service('set_goal', SetBool, self.set_goal_callback)

        # 初始化目标点
        goal_x = rospy.get_param('~goal_x', 0.0)
        goal_y = rospy.get_param('~goal_y', 0.0)
        self.goal_point = Point(goal_x, goal_y, 0)

        rospy.loginfo("Point Navigation Node Initialized")

    def odom_callback(self, msg):
        """处理里程计数据"""
        self.current_pose = msg.pose.pose

    def laser_callback(self, msg):
        """处理激光雷达数据"""
        self.laser_data = msg

    def set_goal_callback(self, req):
        """设置目标点服务回调"""
        if req.data:
            # 从参数服务器获取目标点
            goal_x = rospy.get_param('~goal_x', 0.0)
            goal_y = rospy.get_param('~goal_y', 0.0)
            self.goal_point = Point(goal_x, goal_y, 0)
            self.is_navigating = True
            rospy.loginfo(f"New goal set: ({goal_x}, {goal_y})")
            return SetBoolResponse(success=True, message="Goal set successfully")
        else:
            self.is_navigating = False
            return SetBoolResponse(success=True, message="Navigation stopped")

    def get_distance_to_goal(self):
        """计算到目标点的距离"""
        if self.current_pose is None or self.goal_point is None:
            return float('inf')

        dx = self.goal_point.x - self.current_pose.position.x
        dy = self.goal_point.y - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def get_angle_to_goal(self):
        """计算到目标点的角度"""
        if self.current_pose is None or self.goal_point is None:
            return 0

        dx = self.goal_point.x - self.current_pose.position.x
        dy = self.goal_point.y - self.current_pose.position.y

        # 获取当前朝向
        orientation = self.current_pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        # 计算目标角度
        target_angle = math.atan2(dy, dx)

        # 计算角度差
        angle_diff = target_angle - yaw

        # 归一化角度到[-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        return angle_diff

    def detect_obstacle(self):
        """检测前方是否有障碍物"""
        if self.laser_data is None:
            return False

        # 检查前方扇形区域内的障碍物
        front_angles = 30  # 前方30度范围
        angle_increment = self.laser_data.angle_increment
        center_index = len(self.laser_data.ranges) // 2
        start_index = int(center_index - front_angles / 2 / angle_increment)
        end_index = int(center_index + front_angles / 2 / angle_increment)

        for i in range(start_index, end_index):
            if 0 < i < len(self.laser_data.ranges):
                if self.laser_data.ranges[i] < self.obstacle_threshold:
                    return True

        return False

    def navigate_to_goal(self):
        """导航到目标点"""
        if not self.is_navigating or self.current_pose is None or self.goal_point is None:
            # 停止移动
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        distance = self.get_distance_to_goal()
        angle = self.get_angle_to_goal()

        # 如果到达目标点
        if distance < self.goal_tolerance:
            rospy.loginfo("Goal reached!")
            self.is_navigating = False
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        # 检测障碍物
        if self.detect_obstacle():
            rospy.logwarn("Obstacle detected! Stopping.")
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        # 计算速度
        cmd_vel = Twist()

        # 角度控制
        if abs(angle) > 0.1:
            # 需要转向
            cmd_vel.angular.z = max(min(self.max_angular_speed * angle, self.max_angular_speed), -self.max_angular_speed)
        else:
            # 角度差不多，可以前进
            cmd_vel.linear.x = max(min(self.max_linear_speed * distance, self.max_linear_speed), 0.2)

        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            self.navigate_to_goal()
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = PointNavigation()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
