#!/usr/bin/env python

from typing import List
import rospy
import numpy as np
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
import os 
from racecar_behaviors.srv import BlobList, BlobListResponse
from racecar_behaviors.msg import BlobData
from blob_detector import Blob
from path_finder import PathFinder, Point


class PathFollowing:
    def __init__(self):
        self.distance = 1.5
        self.max_speed = rospy.get_param('~max_speed', 1)
        self.max_steering = rospy.get_param('~max_steering', 0.37)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.blobs: List[Blob] = []
        goals = [(13.5, 2.1, 0.0, 1.0), (12.5, 2.1, 3.1416, 1.0), (0.0, 0.0, 3.1416, 1.0)]
        for goal in goals:
            self.send_to_movebase(goal)
        self.create_report()
        start: Point = Point(0,0)
        path_finder: PathFinder = PathFinder()
        rospy.loginfo(self.blobs)
        for blob in self.blobs:
            goal: Point = Point(blob.robot_x,blob.robot_y)
            path_finder.find_best_path(start, goal, blob.id)

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
        pass
        #rospy.loginfo("Current speed = %f m/s", msg.twist.twist.linear.x)

    def create_report(self):
        rospy.wait_for_service('send_blob_data')
        try:
            get_blob_list = rospy.ServiceProxy('send_blob_data', BlobList)
            blob_response = get_blob_list(1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        else:
            filepath = os.path.join('output_directory', 'Report.txt')
            if not os.path.exists('output_directory'):
                os.makedirs('output_directory')
            
            if os.path.exists(filepath):
                os.remove(filepath)

            with open(filepath, 'a') as file:
                for blob in blob_response.blobs:
                    file.write(f'{blob.x:.2f} {blob.y:.2f} photo_object_{blob.id}.png trajectory_object_{blob.id}.bmp \n')
                    self.blobs.append(blob)
                rospy.loginfo("Report has been created!")

            try:
                os.system("rm -r ~/output_directory")   
            except:
                pass
            os.system("cp -R ~/.ros/output_directory ~/output_directory")

def main():
    rospy.init_node('path_following')
    pathFollowing = PathFollowing()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

