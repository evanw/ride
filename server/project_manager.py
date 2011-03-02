import os, glob
import json
import yaml

class ProjectManager:
    
    def set_workspace(self, path):
        self.workspace_path = os.path.expanduser(path)
        
        try:
            os.makedirs(self.workspace_path)
        except:
            pass
        
        self.projects = os.listdir(self.workspace_path)
        
    def open_project(self, project_name):
        self.current_project = project_name
        
    def get_projects(self, format):
        if format == 'json':
            return json.dumps({"projects": self.projects})