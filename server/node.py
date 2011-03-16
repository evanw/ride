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
        self.update(d)
        self.id = d['id']
        self.inputs = [Connection().from_dict(i) for i in d['inputs']]
        self.outputs = [Connection().from_dict(o) for o in d['outputs']]
        #self.type = d['type']
        return self # allow chaining

    def update(self, d):
        if 'x' in d: self.x = d['x']
        if 'y' in d: self.y = d['y']
        if 'name' in d: self.name = d['name']
