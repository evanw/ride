#!/usr/bin/env python

import os
from channel import Listener
from project_manager import ProjectManager

project_manager = ProjectManager()
project_manager.set_workspace('~/ros_ide/workspace')

class ProjectManagerServer(Listener):
    def __init__(self):
        self.subscribe('workspace', 'list')

    def on_message(self, channel, data):
        if channel == ('workspace', 'list'):
            self.publish(('workspace', 'list'), project_manager.get_projects('json'))

# this will handle requests on a separate thread
pms = ProjectManagerServer()

# serve website on main thread
import staticfileserver
staticfileserver.serve_forever(8000)
