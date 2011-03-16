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
        
        self.projects = [n for n in os.listdir(self.workspace_path) if os.path.exists(os.path.join(self.workspace_path, n, 'project.yaml')) ]
        
    def get_projects(self):
        return {
            'projects': [{ 'name': name } for name in self.projects]
        }
