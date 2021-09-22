#!/usr/bin/env python

from ros_monitor import Serializer
import socket
from struct import *


def int2ip(addr):
    return socket.inet_ntoa(pack("!I", addr))

class vehiculeTracker:
    def __init__(self):
        self.HOST = '255.255.255.255'
        # This process should listen to a different port than the PositionBroadcast client.
        self.PORT = 65431
        self.socket_init()

    def __del__(self):
        try:
            self.vt_socket.close()
        except:
            pass

    def socket_init(self):
        self.vt_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.vt_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.vt_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.vt_socket.bind(("", self.PORT))
    
    def position_receiver(self):
        while True:
            (x, y, theta, id_pos) = Serializer.from_byte_to_info(Serializer.PB_format ,self.vt_socket.recvfrom(1024)[0]) 
            if not id_pos:
                break
            print("VehiculeTracker: ")
            print("ID: ", int2ip(id_pos))
            print("x: ", x)
            print("y: ", y)
            print("theta: ", theta)

         
if __name__=="__main__":
    vehicule = vehiculeTracker()
    while True:
        try:
            vehicule.position_receiver() 
        except Exception as e:
            vehicule.vt_socket.close()
            vehicule.socket_init()
