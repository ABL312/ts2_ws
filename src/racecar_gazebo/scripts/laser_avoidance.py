#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class LaserAvoidance:
    def __init__(self):
        # 1. 初始化节点
        rospy.init_node('laser_avoidance_node')
        
        # 2. 订阅激光雷达话题 /scan
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # 3. 发布速度控制话题 /cmd_vel (配合您之前修改的 launch 文件，不使用重映射)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # 避障参数设置
        self.safety_distance = 0.8  # 触发避障的距离阈值 (米)
        self.linear_speed = 0.5     # 直行速度 (m/s)
        self.turn_speed = 0.5       # 转向时的角速度 (rad/s) [Racecar的转向其实是舵机角度]
        
        rospy.loginfo("Laser Avoidance Node Started. Waiting for scan data...")

    def scan_callback(self, msg):
        # 激光雷达通常有数百个点 (msg.ranges)
        # 假设雷达是水平安装，数组中间的索引对应正前方 (0度)
        
        num_readings = len(msg.ranges)
        mid_index = int(num_readings / 2)
        
        # 取中间一部分范围作为“正前方”检测区
        # 例如：如果总共720个点，我们取中间120个点 (约30度范围)
        window_size = int(num_readings / 6) 
        start_index = mid_index - window_size
        end_index = mid_index + window_size
        
        # 获取前方扇区的数据
        front_ranges = msg.ranges[start_index : end_index]
        
        # 数据清洗：过滤掉 inf (无穷大) 和 nan (无效值)，以及距离过近的噪点
        valid_ranges = []
        for r in front_ranges:
            if not math.isinf(r) and not math.isnan(r) and r > 0.05:
                valid_ranges.append(r)
        
        # 如果没有有效数据，假设前方空旷
        if len(valid_ranges) == 0:
            min_dist = float('inf')
        else:
            min_dist = min(valid_ranges) # 找到前方最近的障碍物距离

        # --- 核心避障逻辑 ---
        twist = Twist()
        
        if min_dist < self.safety_distance:
            # 情况A: 障碍物太近 -> 避障
            rospy.logwarn(f"Obstacle detected! Distance: {min_dist:.2f}m -> Turning Left")
            twist.linear.x = 0.0          # 可以选择减速或停车
            twist.angular.z = self.turn_speed  # 向左转 (正值为左，负值为右)
        else:
            # 情况B: 前方安全 -> 直行
            # rospy.loginfo_throttle(1, f"Path clear. Distance: {min_dist:.2f}m")
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            
        # 发布指令
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    try:
        node = LaserAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass