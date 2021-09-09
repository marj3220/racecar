#!/usr/bin/env python

import socket

HOST = '127.0.0.1'
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

#CEST PAS DU BON CODE
self.rr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
self.rr_socket.bind((self.host, self.remote_request_port))

    while True:
        self.rr_socket.listen(1)
        (conn, addr) = self.rr_socket.accept()
        # RPC Loop
        while True:
            request = conn.recv(1024)
            request = str(request, encoding="ascii")
            print(request)
            if not request: break
            data = bytes("RPC requested is not valid")
            if request == "RPOS":
                data = bytes(self.pos)
            elif request == "OBSF":
                data = bytes(self.obstacle)
            elif request == "RBID":
                data = bytes(self.id)
            conn.send(data)
        conn.close()