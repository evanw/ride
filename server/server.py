#!/usr/bin/env python

import os
from channel import Listener
from project_manager import ProjectManager
from project import *

project_manager = ProjectManager()
project_manager.set_workspace(os.path.join(os.getcwd(), '../workspace'))

class ProjectServer(Listener):
    def __init__(self, path):
        self.project = Project(path)
        self.subscribe('project', self.project.name, 'nodes', 'request')
        self.subscribe('project', self.project.name, 'node', 'update')
        self.subscribe('project', self.project.name, 'node', 'add')
        self.subscribe('project', self.project.name, 'node', 'remove')
        self.subscribe('project', self.project.name, 'node', 'connect')
        self.subscribe('project', self.project.name, 'node', 'disconnect')

    def on_message(self, channel, data):
        if channel == ('project', self.project.name, 'nodes', 'request'):
            self.publish(('project', self.project.name, 'nodes', 'response'), self.project.to_dict())
        elif channel == ('project', self.project.name, 'node', 'update'):
            self.project.update_node(data)
        elif channel == ('project', self.project.name, 'node', 'add'):
            self.project.add_node(data)
        elif channel == ('project', self.project.name, 'node', 'remove'):
            self.project.remove_node(data)
        elif channel == ('project', self.project.name, 'node', 'connect'):
            self.project.add_connection(data)
        elif channel == ('project', self.project.name, 'node', 'disconnect'):
            self.project.remove_connection(data)

class ProjectManagerServer(Listener):
    def __init__(self):
        self.subscribe('workspace', 'list', 'request')
        self.project_servers = [ProjectServer(os.path.join(project_manager.workspace_path, p)) for p in project_manager.projects]

    def on_message(self, channel, data):
        if channel == ('workspace', 'list', 'request'):
            self.publish(('workspace', 'list', 'response'), project_manager.get_projects())

# this will handle requests on a separate thread
pms = ProjectManagerServer()

# serve website on main thread
import staticfileserver
staticfileserver.serve_forever(8000)
