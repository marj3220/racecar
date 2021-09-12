#!/usr/bin/env python

import socket

from ros_monitor import Serializer
from vehicle_tracker import int2ip


class remoteClient:
    def __init__(self):
        self.HOST = '10.0.0.5'
        # This process should listen to a different port than the PositionBroadcast client.
        self.PORT = 65432
        self.rc_socket = None
        self.setConnectionParam()

    def __del__(self):
        try:
            self.rc_socket.close()
        except:
            pass

    def socket_init(self):
        try:
            self.rc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.rc_socket.settimeout(5)
            self.rc_socket.connect((self.HOST, self.PORT))
            self.rc_socket.settimeout(None)
        except:
            print("Connection failed: Host and/or port are invalid. Please try again.")
            self.PORT = 65432
            self.HOST = '10.0.0.5'
            self.setConnectionParam()
    
    def showMainMenu(self):
        while True:
            print("Drop-down menu: ")
            print("1. Set Connection Params (Already connected to default vehicule)")
            print("2. Send Command to Server")
            print("3. Close connection")
            try:
                menu_option = int(input("Select from drop-down: "))
                self.executeMainMenu(menu_option)
            except:
                print("Menu option was not a number")

    def executeMainMenu(self, main_menu_request):
        if main_menu_request == 1:
            self.setConnectionParam()
        elif main_menu_request == 2:
            self.menuCmd()
        elif main_menu_request == 3:
            self.rc_socket.close()
            print("Connection to host has been closed")
        else:
            print("Invalid request")

    def setConnectionParam(self):
        if self.rc_socket:
            self.rc_socket.close()
        self.temp_host = str(input("Enter HOST IP Address (Press ENTER for default): "))
        if self.temp_host != '':
            self.HOST = self.temp_host
        self.temp_port = str(input("Enter Port number (Press ENTER for default): "))
        if self.temp_port != '':
            self.PORT = int(self.temp_port)
        self.socket_init()

    def menuCmd(self):
        # RPC Loop
        print("1. RPOS")
        print("2. OBSF")
        print("3. RBID")
        RPC_Request = int(input("Input wanted RPC's number: "))
        try:
            self.sendCmd(RPC_Request)
        except:
            print("Socket connection is invalid, returning to main menu. Please set connection params.")
    
    def sendCmd(self, RPC_Request):
        if RPC_Request == 1:
            data = str.encode("RPOS")
            format = Serializer.RPOS_format
        elif RPC_Request == 2:
            data = str.encode("OBSF")
            format = Serializer.OBSF_RBID_format
        elif RPC_Request == 3:
            data = str.encode("RBID")
            format = Serializer.OBSF_RBID_format
        else:
            print("Invalid request")
            return
        self.rc_socket.send(data)
        data = self.rc_socket.recv(1024)
        decoded_data = Serializer.from_byte_to_info(format, data)
        print("Received data:")
        self.print_data(decoded_data, RPC_Request)

    def print_data(self, decoded_data, rpc_request_num):
        if rpc_request_num == 1:
            for data in decoded_data:
                print(str(data))
        elif rpc_request_num == 2:
            print(bool(decoded_data[0]))
        elif rpc_request_num == 3:
            print(int2ip(decoded_data[0]))

if __name__=="__main__":
    client = remoteClient()
    client.showMainMenu()
