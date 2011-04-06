import os
import yaml
from node import Node

class Library:
    def __init__(self, path):
        self.path = path
        self.library_file_path = os.path.join(self.path, 'library.yaml')
        self.load()

    def load(self):
        self.dict = yaml.safe_load(file(self.library_file_path, 'r'))
        self.nodes = [Node().from_dict(n) for n in self.dict['nodes']]

    def to_dict(self):
        return self.dict
