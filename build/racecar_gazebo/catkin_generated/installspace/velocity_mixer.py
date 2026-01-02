#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse

class VelocityMixer:
    def __init__(self):
        rospy.init_node('velocity_mixer', anonymous=True)

        # 初始化参数
        self.avoidance_priority = rospy.get_param('~avoidance_priority', True)  # 避障是否优先

        # 当前速度指令
        self.navigation_cmd = Twist()
        self.avoidance_cmd = Twist()

        # 订阅话题
        rospy.Subscriber('navigation_cmd_vel', Twist, self.navigation_callback)
        rospy.Subscriber('avoidance_cmd_vel', Twist, self.avoidance_callback)

        # 发布话题
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # 提供服务
        self.toggle_service = rospy.Service('toggle_priority', SetBool, self.toggle_callback)

        rospy.loginfo("Velocity Mixer Node Initialized")

    def navigation_callback(self, msg):
        """处理导航速度指令"""
        self.navigation_cmd = msg

    def avoidance_callback(self, msg):
        """处理避障速度指令"""
        self.avoidance_cmd = msg

    def toggle_callback(self, req):
        """切换优先级"""
        self.avoidance_priority = req.data
        if req.data:
            rospy.loginfo("Avoidance has priority")
            return SetBoolResponse(success=True, message="Avoidance has priority")
        else:
            rospy.loginfo("Navigation has priority")
            return SetBoolResponse(success=True, message="Navigation has priority")

    def mix_velocities(self):
        """混合速度指令"""
        # 如果避障有优先级且避障指令非零，则使用避障指令
        if self.avoidance_priority and (abs(self.avoidance_cmd.linear.x) > 0.01 or abs(self.avoidance_cmd.angular.z) > 0.01):
            return self.avoidance_cmd
        # 否则使用导航指令
        else:
            return self.navigation_cmd

    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            # 混合速度指令
            cmd_vel = self.mix_velocities()

            # 发布混合后的速度指令
            self.cmd_vel_pub.publish(cmd_vel)

            rate.sleep()

if __name__ == '__main__':
    try:
        mixer = VelocityMixer()
        mixer.run()
    except rospy.ROSInterruptException:
        pass
