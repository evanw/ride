class Connection:
    def __init__(self):
        self.type = None
        self.name = ''
        self.node_ids = []

    def to_dict(self):
        return {
            'type': self.type,
            'name': self.name,
            'node_ids': self.node_ids
        }

    def from_dict(self, d):
        self.type = d['type']
        self.name = d['name']
        self.node_ids = d['node_ids']
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
        self.type = d['type']
        return self # allow chaining
