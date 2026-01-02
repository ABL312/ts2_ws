#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelStates

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('racecar_odometry', anonymous=True)

        # 参数
        self.robot_name = rospy.get_param('~robot_name', 'racecar')
        self.publish_tf = rospy.get_param('~publish_tf', True)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        # 发布者
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

        # TF广播器
        if self.publish_tf:
            self.tf_broadcaster = tf.TransformBroadcaster()

        # 订阅者
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        # 初始化变量
        self.last_time = rospy.Time.now()
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0

        rospy.loginfo("Odometry Publisher Initialized")

    def model_states_callback(self, msg):
        """处理模型状态消息"""
        try:
            # 找到机器人模型
            index = msg.name.index(self.robot_name)

            # 获取当前位置和方向
            pose = msg.pose[index]
            twist = msg.twist[index]

            # 计算欧拉角
            orientation = pose.orientation
            _, _, yaw = tf.transformations.euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w])

            # 创建里程计消息
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame

            # 设置位置
            odom.pose.pose.position.x = pose.position.x
            odom.pose.pose.position.y = pose.position.y
            odom.pose.pose.position.z = pose.position.z
            odom.pose.pose.orientation = pose.orientation

            # 设置速度
            odom.twist.twist.linear.x = twist.linear.x
            odom.twist.twist.linear.y = twist.linear.y
            odom.twist.twist.linear.z = twist.linear.z
            odom.twist.twist.angular.x = twist.angular.x
            odom.twist.twist.angular.y = twist.angular.y
            odom.twist.twist.angular.z = twist.angular.z

            # 发布里程计
            self.odom_pub.publish(odom)

            # 发布TF变换
            if self.publish_tf:
                self.tf_broadcaster.sendTransform(
                    (pose.position.x, pose.position.y, pose.position.z),
                    (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                    rospy.Time.now(),
                    self.base_frame,    # child frame
                    self.odom_frame     # parent frame
                )

              



        except (ValueError, IndexError):
            # 如果找不到机器人模型，忽略
            pass

    def run(self):
        """主循环"""
        rospy.spin()

if __name__ == '__main__':
    try:
        odom_publisher = OdometryPublisher()
        odom_publisher.run()
    except rospy.ROSInterruptException:
        pass
