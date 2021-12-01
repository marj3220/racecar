#!/usr/bin/env python

import rospy
import numpy as np
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult


class PathFollowing:
    def __init__(self):
        self.distance = 1.5
        self.max_speed = rospy.get_param('~max_speed', 1)
        self.max_steering = rospy.get_param('~max_steering', 0.37)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        goals = [(13.5, 2.1, 0.0, 1.0), (12.5, 2.1, -90.0, 1.0), (0.0, 0.0, 180.0, 1.0)]
        for goal in goals:
            self.send_to_movebase(goal)

    def send_to_movebase(self, unchecked_goal):
        self.client.wait_for_server()
        goal: MoveBaseGoal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "racecar/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = unchecked_goal[0]
        goal.target_pose.pose.position.y = unchecked_goal[1]
        goal.target_pose.pose.orientation.z = unchecked_goal[2]
        goal.target_pose.pose.orientation.w = unchecked_goal[3]
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()  
      
    def scan_callback(self, msg):
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        l2 = int(len(msg.ranges)/2)
        ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]
        
        # Obstacle front?
        obstacleDetected = False
        for i in range(l2-int(l2/12), l2+int(l2/12)) :
            if np.isfinite(ranges[i]) and ranges[i]>0:
                
                break
                np.mean()
                
        if obstacleDetected:
            self.cmd_vel_pub.publish(Twist()); # zero twist  
            rospy.loginfo("Obstacle detected! Stop!")  
        
        twist = Twist()
        twist.linear.x = self.max_speed
        twist.angular.z = 0
           
        #self.cmd_vel_pub.publish(twist)
        
    def odom_callback(self, msg):
        rospy.loginfo("Current speed = %f m/s", msg.twist.twist.linear.x)

def main():
    rospy.init_node('path_following')
    pathFollowing = PathFollowing()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

