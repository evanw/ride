#!/usr/bin/env python

import os
from channel import Listener
from project_manager import ProjectManager

project_manager = ProjectManager()
project_manager.set_workspace('~/ros_ide/workspace')

class ProjectServer(Listener):
    def __init__(self, name):
        self.name = name
        self.subscribe('project', self.name, 'nodes', 'request')

    def on_message(self, channel, data):
        if channel == ('project', self.name, 'nodes', 'request'):
            self.publish(('project', self.name, 'nodes', 'response'), project_manager.get_project(self.name).get_nodes())

class ProjectManagerServer(Listener):
    def __init__(self):
        self.subscribe('workspace', 'list', 'request')
        self.project_servers = []

    def on_message(self, channel, data):
        if channel == ('workspace', 'list', 'request'):
            self.publish(('workspace', 'list', 'response'), project_manager.get_projects())

# this will handle requests on a separate thread
pms = ProjectManagerServer()

# serve website on main thread
import staticfileserver
staticfileserver.serve_forever(8000)
