import os, glob
import re
import json
import yaml
from library import *
from project import *
from channel import Listener
from deploythread import DeployThread

class ProjectServer(Listener):
    def __init__(self, path):
        self.deploy_thread = None
        self.project = Project(path)
        self.subscribe('project', self.project.name, 'nodes', 'request')
        self.subscribe('project', self.project.name, 'node', 'update')
        self.subscribe('project', self.project.name, 'node', 'add')
        self.subscribe('project', self.project.name, 'node', 'remove')
        self.subscribe('project', self.project.name, 'node', 'connect')
        self.subscribe('project', self.project.name, 'node', 'disconnect')
        self.subscribe('project', self.project.name, 'deploy', 'run')
        self.subscribe('project', self.project.name, 'deploy', 'stop')

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
        elif channel == ('project', self.project.name, 'deploy', 'run'):
            self.run(data)
        elif channel == ('project', self.project.name, 'deploy', 'stop'):
            self.stop()

    def run(self, json):
        self.stop()
        self.deploy_thread = DeployThread(json['ip'], json['user'], json['pass'], self.project)
        self.deploy_thread.start()

    def stop(self):
        if self.deploy_thread:
            self.deploy_thread.kill()
            self.deploy_thread = None

def makedirs(dirs):
    try:
        os.makedirs(dirs)
    except:
        pass

class ProjectManager:
    
    def set_workspace(self, path):
        makedirs(path)
        self.workspace_path = path
        self.library = Library(path)
        self.projects = [n for n in os.listdir(self.workspace_path) if os.path.exists(os.path.join(self.workspace_path, n, 'project.yaml')) ]
        self.project_servers = [ProjectServer(os.path.join(path, p)) for p in self.projects]

    def add_project(self, name):
        name = name.strip()
        if not re.match(r'^\w+$', name):
            # error: invalid name
            return

        # make a new project with the given name
        if name not in self.projects:
            project_path = os.path.join(self.workspace_path, name)
            makedirs(project_path)
            open(os.path.join(project_path, 'project.yaml'), 'w').write('nodes: []\n')
            self.projects.append(name)
            self.project_servers.append(ProjectServer(project_path))

    def get_projects(self):
        return {
            'projects': [{ 'name': name } for name in self.projects]
        }

    def get_library(self):
        return self.library.to_dict()
