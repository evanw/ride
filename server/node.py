class Connection:
    def __init__(self):
        self.id = None
        self.type = None
        self.name = ''
        self.connections = []

    def to_dict(self):
        return {
            'id': self.id,
            'type': self.type,
            'name': self.name,
            'connections': self.connections
        }

    def from_dict(self, d):
        if 'id' in d:
            self.id = d['id']
        if 'type' in d:
            self.type = d['type']
        if 'name' in d:
            self.name = d['name']
        if 'connections' in d:
            self.connections = d['connections']
        return self # allow chaining

class Node:
    def __init__(self):
        self.id = 0 # TODO: increment next node id
        self.x = 0
        self.y = 0
        self.name = ''
        self.inputs = []
        self.outputs = []
        self.type = None
        # TODO: details about repo it came from and package/stack it's in

    def to_dict(self):
        return {
            'id': self.id,
            'x': self.x,
            'y': self.y,
            'name': self.name,
            'inputs': [i.to_dict() for i in self.inputs],
            'outputs': [o.to_dict() for o in self.outputs],
            'type': self.type
        }

    def from_dict(self, d):
        self.id = d['id']
        self.x = d['x']
        self.y = d['y']
        self.name = d['name']
        self.inputs = [Connection().from_dict(i) for i in d['inputs']]
        self.outputs = [Connection().from_dict(o) for o in d['outputs']]
        #self.type = d['type']
        return self # allow chaining


    def diff(self, other):
        if self.id is not other.id:
            return True
        if self.x is not other.x:
            return True
        if self.y is not other.y:
            return True
        if self.name is not other.name:
            return True
        if self.inputs is not other.inputs:
            return True
        if self.outputs is not other.outputs:
            return True
        return False