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
        
        
    def get_projects(self, format):
        return {
            'projects': [{ 'name': name } for name in self.projects]
        }

    