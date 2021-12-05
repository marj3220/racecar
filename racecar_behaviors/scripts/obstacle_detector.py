#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class ObstacleDetector:
    def __init__(self):
        self.going_in_reverse = False
        self.distance = rospy.get_param('~distance', 0.75)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)

    def odom_callback(self, msg: Odometry):
        if msg.twist.twist.linear.x < 0:
            self.going_in_reverse = True
        else:
            self.going_in_reverse = False

    def scan_callback(self, msg: LaserScan):
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        # Obstacle front?
        l2 = int(len(msg.ranges)/2)
        ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]
        obstacleDetectedFront = False
        for i in range(l2-int(l2/8), l2+int(l2/8)) :
            if np.isfinite(ranges[i]) and ranges[i]>0 and ranges[i] < self.distance:
                obstacleDetectedFront = True
                break
        # Obstacle back?
        ranges = msg.ranges
        obstacleDetectedBack = False
        for i in range(l2-int(l2/8), l2+int(l2/8)) :
            if np.isfinite(ranges[i]) and ranges[i]>0 and ranges[i] < self.distance:
                obstacleDetectedBack = True
                break

        if obstacleDetectedFront:
            self.cmd_vel_pub.publish(Twist()); # zero twist
            rospy.loginfo("Obstacle detected! Stop!")
            if obstacleDetectedBack == False:
                obstacle_path = Twist()
                obstacle_path.linear.x = -1.0
                self.cmd_vel_pub.publish(obstacle_path)
            else:
                rospy.loginfo("Dynamic obstacle detected behind the robot! Waiting for obstacle to pass!")

        if obstacleDetectedBack and self.going_in_reverse:
            self.cmd_vel_pub.publish(Twist()); # zero twist
            rospy.loginfo("Dynamic obstacle detected behind the robot! Waiting for obstacle to pass!")


def main():
    rospy.init_node('obstacle_detector')
    obstacleDetector = ObstacleDetector()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

