#!/usr/bin/env python

import socket

from ros_monitor import Serializer


class remoteClient:
    def __init__(self):
        self.HOST = '127.0.0.1'
        # This process should listen to a different port than the PositionBroadcast client.
        self.PORT = 65432
        self.socket_init()

    def __del__(self):
        try:
            self.rc_socket.close()
        except:
            pass

    def socket_init(self):
        self.rc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rc_socket.connect((self.HOST, self.PORT))
    
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
            self.rc_socket.close
        else:
            print("Invalid request")

    def setConnectionParam(self):
        self.rc_socket.close()
        self.HOST = str(input("Enter HOST IP Address:"))
        self.PORT = str(input("Enter Port number :"))
        self.socket_init()

    def menuCmd(self):
        # RPC Loop
        print("1. RPOS")
        print("2. OBSF")
        print("3. RBID")
        RPC_Request = int(input("Input wanted RPC's number: "))
        try:
            self.sendCmd(RPC_Request)
        except Exception as e:
            print("Socket has timed out, retrying connection, error: ", e)
            self.socket_init()
    
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
        for data in decoded_data:
            print(str(data))

if __name__=="__main__":
    client = remoteClient()
    client.showMainMenu()
