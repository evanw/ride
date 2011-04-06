class Connection:
    def __init__(self):
        self.id = None
        self.type = None
        self.name = ''
        self.connections = []

    def to_dict(self):
        return {
            'id': str(self.id),
            'type': self.type,
            'name': self.name,
            'connections': map(str, self.connections),
        }

    def from_dict(self, d):
        if 'id' in d: self.id = str(d['id'])
        if 'type' in d: self.type = d['type']
        if 'name' in d: self.name = d['name']
        if 'connections' in d: self.connections = map(str, d['connections'])
        return self # allow chaining

class Param:
    def __init__(self):
        self.name = ''
        self.type = None
        self.value = None

    def to_dict(self):
        return {
            'name': self.name,
            'type': self.type,
            'value': self.value,
        }

    def from_dict(self, d):
        if 'name' in d: self.name = d['name']
        if 'type' in d: self.type = d['type']
        if 'value' in d: self.value = d['type']
        return self # allow chaining

EXEC_ROSLAUNCH = 0
EXEC_BINARY = 1

class Node:
    def __init__(self):
        self.id = ''
        self.x = 0
        self.y = 0
        self.name = ''
        self.inputs = []
        self.outputs = []
        self.params = []
        
        # what package this node came from
        self.pkg = ''
        
        # what to run in self.pkg
        self.exec_name = ''
        
        # how to run self.exec_name:
        # - EXEC_ROSLAUNCH to run with roslaunch
        # - EXEC_BINARY to run with bash
        # - None to indicate that self.exec_name is invalid
        self.exec_mode = None
        
        # - True to run the node in the package directory
        # - False to run the node in the ROS root directory
        self.chdir = False

        # TODO: will be done automatically in the future
        # list of things to remap, just including this to not clobber the existing example file
        self.remap = []

    def to_dict(self):
        d = {
            'id': str(self.id),
            'x': self.x,
            'y': self.y,
            'name': self.name,
            'inputs': [i.to_dict() for i in self.inputs],
            'outputs': [o.to_dict() for o in self.outputs],
            'params': [p.to_dict() for p in self.params],
            'pkg': self.pkg,
            'chdir': self.chdir,
            'remap': self.remap,
        }
        if self.exec_mode == EXEC_ROSLAUNCH: d['launch'] = self.exec_name
        elif self.exec_mode == EXEC_BINARY: d['exec'] = self.exec_name
        return d

    def from_dict(self, d):
        self.update(d)
        if 'id' in d: self.id = str(d['id'])
        if 'inputs' in d: self.inputs = [Connection().from_dict(i) for i in d['inputs']]
        if 'outputs' in d: self.outputs = [Connection().from_dict(o) for o in d['outputs']]
        if 'params' in d: self.params = [Param().from_dict(p) for p in d['params']]
        if 'exec' in d:
            self.exec_mode = EXEC_BINARY
            self.exec_name = d['exec']
        elif 'launch' in d:
            self.exec_mode = EXEC_ROSLAUNCH
            self.exec_name = d['launch']
        return self # allow chaining

    def update(self, d):
        if 'x' in d: self.x = d['x']
        if 'y' in d: self.y = d['y']
        if 'name' in d: self.name = d['name']
        if 'chdir' in d: self.chdir = d['chdir']
        if 'remap' in d: self.remap = d['remap']
        if 'pkg' in d: self.pkg = d['pkg']
