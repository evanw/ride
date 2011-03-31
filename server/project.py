import os
import yaml
from node import Node

class Project:
    def __init__(self, path):
        self.name = os.path.basename(path)
        self.setPath(path)
        self.nodes = []
        self.load()
        
    def setPath(self, p):
        self.path = p
        self.project_file_path = os.path.join(self.path, 'project.yaml')

    def add_node(self, node):
        node.id = self.getNextId()
        self.nodes.append(node)
        self.save()

    def update_node(self, json):
        for n in self.nodes:
            if n.id == json['id']:
                n.update(json)
                break
        self.save()

    def add_node(self, json):
        pass

    def remove_node(self, json):
        self.nodes = [n for n in self.nodes if n.id != json['id']]
        self.remove_invalid_connections()
        self.save()

    def add_connection(self, json):
        input_id, output_id = json['input'], json['output']
        for n in self.nodes:
            for i in n.inputs:
                if i.id == input_id and output_id not in i.connections:
                    i.connections.append(output_id)
            for o in n.outputs:
                if o.id == output_id and input_id not in o.connections:
                    o.connections.append(input_id)
        self.remove_invalid_connections()
        self.save()

    def remove_connection(self, json):
        input_id, output_id = json['input'], json['output']
        for n in self.nodes:
            for i in n.inputs:
                if i.id == input_id and output_id in i.connections:
                    i.connections.remove(output_id)
            for o in n.outputs:
                if o.id == output_id and input_id in o.connections:
                    o.connections.remove(input_id)
        self.remove_invalid_connections()
        self.save()
        
    def to_dict(self):
        return {
            'nodes': [x.to_dict() for x in self.nodes]
        }
        
    def save(self):
        handle = open(self.project_file_path, 'w')
        handle.write(yaml.safe_dump(self.to_dict()))
        handle.close()
        
    def load(self):
        yml = yaml.safe_load(file(self.project_file_path, 'r'))
        self.nodes = [Node().from_dict(n) for n in yml['nodes']]
        self.remove_invalid_connections()
        self.save()

    def remove_invalid_connections(self):
        valid_input_ids = dict((c.id, c) for n in self.nodes for c in n.inputs)
        valid_output_ids = dict((c.id, c) for n in self.nodes for c in n.outputs)
        
        # remove all inputs and outputs with invalid connection ids
        for node in self.nodes:
            for i in node.inputs:
                for id in i.connections[:]:
                    if id not in valid_output_ids:
                        print 'warning: removing %s->%s from %s.%s (%s is an invalid output id)' % (i.id, id, node.name, i.name, id)
                        i.connections.remove(id)
            for o in node.outputs:
                for id in o.connections[:]:
                    if id not in valid_input_ids:
                        print 'warning: removing %s->%s from %s.%s (%s is an invalid input id)' % (o.id, id, node.name, o.name, id)
                        o.connections.remove(id)
        
        # remove all inputs and outputs that don't reference each other
        for node in self.nodes:
            for i in node.inputs:
                for id in i.connections[:]:
                    if i.id not in valid_output_ids[id].connections:
                        print 'warning: removing %s->%s from %s.%s (%s didn\'t point back to %s)' % (i.id, id, node.name, i.name, id, i.id)
                        i.connections.remove(id)
            for o in node.outputs:
                for id in o.connections[:]:
                    if o.id not in valid_input_ids[id].connections:
                        print 'warning: removing %s->%s from %s.%s (%s didn\'t point back to %s)' % (o.id, id, node.name, o.name, id, o.id)
                        o.connections.remove(id)
