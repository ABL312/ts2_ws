#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from std_srvs.srv import SetBool

def set_goal(x, y):
    """设置目标点"""
    rospy.init_node('set_goal_client', anonymous=True)

    # 调用设置目标点服务
    try:
        rospy.wait_for_service('/point_navigation_node/set_goal', 5)
        set_goal_service = rospy.ServiceProxy('/point_navigation_node/set_goal', SetBool)

        # 设置目标点参数
        rospy.set_param('/point_navigation_node/goal_x', x)
        rospy.set_param('/point_navigation_node/goal_y', y)

        response = set_goal_service(True)

        if response.success:
            rospy.loginfo(f"Goal set to ({x}, {y}): {response.message}")
        else:
            rospy.logerr(f"Failed to set goal: {response.message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: set_goal.py [x] [y]")
        sys.exit(1)

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        set_goal(x, y)
    except ValueError:
        print("Error: x and y must be numbers")
        sys.exit(1)
