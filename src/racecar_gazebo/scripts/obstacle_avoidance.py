#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # --- 参数调优 ---
        self.drive_speed = 0.55      # 前进速度
        self.reverse_speed = -0.4  # 倒车速度
        self.turn_speed = 1.5       # 转向力度
        
        # 距离阈值
        self.DANGER_DIST = 0.35 # 前方危险距离 (开始倒车)
        self.SAFE_DIST = 0.9      # 前方安全距离 (停止倒车)
        self.REAR_DIST = 0.4        # 后方危险距离 (屁股要撞了!)
        
        self.is_reversing = False   # 倒车状态标志

        self.target_angle = 0.0
        self.current_angle = 0.0
        self.SMOOTH_FACTOR = 0.75

        rospy.loginfo("Safe Obstacle Avoidance Started (Rear Protection On)")

    def scan_callback(self, msg):
        cmd = Twist()
        ranges = msg.ranges
        count = len(ranges)
        mid = int(count / 2)
        
        # 1. 视觉处理：看前面
        window = int(count / 6) # 60度窗口
        front_ranges = ranges[mid - int(window/2) : mid + int(window/2)]
        valid_front = [r for r in front_ranges if not math.isinf(r) and r > 0.01]
        min_front = min(valid_front) if valid_front else 10.0

        # 2. 视觉处理：看后面 (Lidar数据索引0和末尾是正后方)
        # 取后面左右各 10 度
        rear_window = int(count / 18) 
        rear_ranges = ranges[0:rear_window] + ranges[count-rear_window:count]
        valid_rear = [r for r in rear_ranges if not math.isinf(r) and r > 0.01]
        min_rear = min(valid_rear) if valid_rear else 10.0

        # 3. 视觉处理：看左右 (决定往哪打方向)
        # 3. 视觉处理：看左右 (包含防无穷大处理)
        # 使用多个扇区进行加权计算，提高障碍物识别精度
        # 左侧：分为前中后三个扇区
        left_front = ranges[mid : mid + int(count/8)]
        left_mid = ranges[mid + int(count/8) : mid + int(count/4)]
        left_rear = ranges[mid + int(count/4) : mid + int(count/3)]
        
        # 右侧：分为前中后三个扇区
        right_front = ranges[mid - int(count/8) : mid]
        right_mid = ranges[mid - int(count/4) : mid - int(count/8)]
        right_rear = ranges[mid - int(count/3) : mid - int(count/4)]
        
        # 处理无效数据
        def clean_slice(slice_data):
            return [r if not math.isinf(r) and r > 0.01 else 10.0 for r in slice_data]
        
        clean_left_front = clean_slice(left_front)
        clean_left_mid = clean_slice(left_mid)
        clean_left_rear = clean_slice(left_rear)
        clean_right_front = clean_slice(right_front)
        clean_right_mid = clean_slice(right_mid)
        clean_right_rear = clean_slice(right_rear)

        # 加权计算：前方权重最大，因为那是我们要去的方向
        # 前方权重3，中间权重2，后方权重1
        mean_left = (sum(clean_left_front)*3 + sum(clean_left_mid)*2 + sum(clean_left_rear)*1) / \
                   (len(clean_left_front)*3 + len(clean_left_mid)*2 + len(clean_left_rear)*1)
        mean_right = (sum(clean_right_front)*3 + sum(clean_right_mid)*2 + sum(clean_right_rear)*1) / \
                    (len(clean_right_front)*3 + len(clean_right_mid)*2 + len(clean_right_rear)*1)
        # --- 决策逻辑 ---

        # 紧急情况：倒车时屁股要撞了！
        if self.is_reversing and min_rear < self.REAR_DIST:
            rospy.logwarn_throttle(0.5, "Rear Crash Risk! Stopping Reverse.")
            self.is_reversing = False # 强制退出倒车模式，尝试向前蠕动
            cmd.linear.x = 0.1 # 稍微向前顶一下
            # 往宽敞的地方使劲打轮
            cmd.angular.z = self.turn_speed if mean_left > mean_right else -self.turn_speed
        
        # 正常倒车逻辑
        elif self.is_reversing:
            if min_front > self.SAFE_DIST:
                self.is_reversing = False # 倒够了，前面宽敞了
                rospy.loginfo("Reversing Complete. Moving Forward.")
            else:
                # 继续倒车
                cmd.linear.x = self.reverse_speed
                # 倒车时反向打轮 (反直觉：想车头左摆，需往左打轮让屁股右摆? 
                # 实际上阿克曼倒车：方向盘左打->车倒退向左后->车头相对向右甩。
                # 这里的逻辑：如果左边宽敞，我们想让车头往左转，所以倒车时应该让屁股往右去，
                # 屁股往右去 = 方向盘往左打。
                cmd.angular.z = self.turn_speed if mean_left > mean_right else -self.turn_speed

        # 前进逻辑
        else:
            if min_front < self.DANGER_DIST:
                self.is_reversing = True # 遇到障碍，切入倒车
                cmd.linear.x = 0.0
                rospy.loginfo("Obstacle Ahead! Reversing...")
            else:
                cmd.linear.x = self.drive_speed
                # 简化避障逻辑：只关注避障，不追求居中
                diff = mean_left - mean_right
                
                # 设置死区：只有当差异超过阈值时才转向
                deadzone = 0.3
                if abs(diff) < deadzone:
                    raw_angle = 0.0
                else:
                    # 根据前方距离调整转向力度
                    if min_front < 1.0:
                        gain = 0.8  # 距离近时，转向更灵敏
                    else:
                        gain = 0.4  # 距离远时，转向更温和
                    raw_angle = diff * gain
                

                
                # 限制最大转向角度
                raw_angle = max(min(raw_angle, 1.0), -1.0)
                
                # 使用平滑算法
                self.current_angle = self.current_angle * 0.6 + raw_angle * 0.4
                cmd.angular.z = self.current_angle

        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass