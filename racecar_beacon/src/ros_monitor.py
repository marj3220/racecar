#!/usr/bin/env python

import socket
from struct import *
import threading

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


def quaternion_to_yaw(quat) -> float:
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    # yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (_, _, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw

class Serializer:
    RPOS_format = "fffxxxxx"
    OBSF_RBID_format = "Ixxxxxxxxxxxx"

    @staticmethod
    def from_info_to_byte_RPOS(data):
        return pack(Serializer.RPOS_format, data[0], data[1], data[2])

    @staticmethod
    def from_info_to_byte_OBSF_RBID(data):
        return pack(Serializer.OBSF_RBID_format, data)
    
    @staticmethod
    def from_byte_to_info(format: str, encoded_data):
        return unpack(format, encoded_data)

class ROSMonitor:
    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
        self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.obstacle_cb)
        # Current robot state:
        self.id = 0xFFFF
        self.pos = (0,0,0)
        self.obstacle = False
        # Host:
        self.host = '127.0.0.1'
        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)
        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.remote_request_loop)
        # Thread for PositionBroadcast handling
        self.pb_thread = threading.Thread(target=self.position_broadcast_loop)
        print("ROSMonitor started.")
        self.rr_thread.start()
        self.pb_thread.start()
        self.rr_thread.join()
        self.pb_thread.join()
    
    def remote_request_loop(self):
        # Init of socket
        self.rr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rr_socket.bind((self.host, self.remote_request_port))
        self.rr_socket.settimeout(99999)
        print("RemoteRequest Service has started")
        while True:
            self.rr_socket.listen(1)
            (conn, _) = self.rr_socket.accept()
            print("Connected to client")
            # RPC Loop
            while True:
                request = conn.recv(1024)
                if not request: break
                request = request.decode()
                print(request)
                data = str.encode("RPC requested is not valid")
                if request == "RPOS":
                    data = Serializer.from_info_to_byte_RPOS(self.pos)
                elif request == "OBSF":
                    data = Serializer.from_info_to_byte_OBSF_RBID(self.obstacle)
                elif request == "RBID":
                    data = Serializer.from_info_to_byte_OBSF_RBID(self.id)
                conn.send(data)
            conn.close()

    def position_broadcast_loop(self):
        # Init of socket
        #self.pb_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self.pb_socket.connect((self.host, self.pos_broadcast_port))
        # RPC Loop
        while True:
            pass

    # Odometry Callback:
    def odom_cb(self, msg: Odometry):
        temp_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, quaternion_to_yaw(msg.pose.pose.orientation))
        self.pos = temp_pos

    # Obstacble Callback: 
    def obstacle_cb(self, msg: LaserScan):
        temp_obstacle = not(all(range>1 for range in msg.ranges))
        self.obstacle = temp_obstacle



if __name__=="__main__":
    rospy.init_node("ros_monitor")

    node = ROSMonitor()

    rospy.spin()
