#!/usr/bin/env python

import os
from channel import Listener
from project_manager import ProjectManager
from library import *

workspace_path = os.path.join(os.getcwd(), '../workspace')

project_manager = ProjectManager()
project_manager.set_workspace(workspace_path)

class ProjectManagerServer(Listener):
    def __init__(self):
        self.subscribe('workspace', 'list', 'add')
        self.subscribe('workspace', 'list', 'request')
        self.subscribe('workspace', 'library', 'request')

    def on_message(self, channel, data):
        if channel == ('workspace', 'list', 'request'):
            self.publish(('workspace', 'list', 'response'), project_manager.get_projects())
        elif channel == ('workspace', 'library', 'request'):
            self.publish(('workspace', 'library', 'response'), project_manager.get_library())
        elif channel == ('workspace', 'list', 'add'):
            project_manager.add_project(data['name'])

# this will handle requests on a separate thread
pms = ProjectManagerServer()

# serve website on main thread
import staticfileserver
staticfileserver.serve_forever(8000)
