#!/usr/bin/env python

import os
import SocketServer
from message import *
from project_manager import *

class MyUDPHandler(SocketServer.BaseRequestHandler):

    def handle(self):
        data = self.request[0].strip()
        m = Message(data)
        socket = self.request[1]
        if m.action == 'GET' and m.path == 'projects':
            socket.sendto(project_manager.get_projects('json'), (self.client_address[0], SEND_PORT))
            return

project_manager = ProjectManager()
project_manager.set_workspace('~/ros_ide/workspace')

if __name__ == "__main__":
    HOST, LISTEN_PORT, SEND_PORT = "localhost", 5002, 5001
    server = SocketServer.UDPServer((HOST, LISTEN_PORT), MyUDPHandler)
    server.serve_forever()