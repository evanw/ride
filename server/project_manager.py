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
        if format == 'json':
            return {
                'projects': [{ 'name': name } for name in self.projects]
            }

    def read_project_file(self, project):
        path = os.path.join(self.workspace_path, project, 'project.yaml')
        return yaml.safe_load(file(path, 'r'))
        
    def add_node_to_project(self, project, node):
        yml = self.read_project_file(project)
        yml['nodes'][node.keys()[0]] = node[node.keys()[0]]
        path = os.path.join(self.workspace_path, project, 'project.yaml')
        handle = open(path, 'w')
        handle.write(yaml.dump(yml))
        handle.close()
        
    def update_node_in_project(self, project, node):
        self.add_node_to_porject(project, node)