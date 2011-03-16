import os
import yaml
from node import Node

class Project:
    def __init__(self, path):
        self.name = os.path.basename(path)
        self.setPath(path)
        self.nodes = []
        self.next_id = 0
        self.load()
        
    def setPath(self, p):
        self.path = p
        self.project_file_path = os.path.join(self.path, 'project.yaml')

    def add_node(self, node):
        node.id = self.getNextId()
        self.nodes.append(node)
        self.save()

    def update_node(self, node):
        for i, n in enumerate(self.nodes):
            if n.id == node.id:
                self.nodes[i] = node
                break
        self.save()
        
    def to_dict(self):
        return {
            'nodes': [x.to_dict() for x in self.nodes]
        }
        
    def save(self):
        handle = open(self.project_file_path, 'w')
        handle.write(yaml.dump(self.to_dict()))
        handle.close()
        
    def load(self):
        yml = yaml.safe_load(file(self.project_file_path, 'r'))
        self.nodes = [Node().from_dict(n) for n in yml['nodes']]
        self.next_id = max(n.id for n in self.nodes) + 1
        
    def getNextId(self):
        i = self.next_id
        self.next_id += 1
        return i
