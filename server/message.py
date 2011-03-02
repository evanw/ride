import json

class Message:
    def __init__(self, data):
        obj = json.loads(data)
        self.action = obj['action']
        self.path = obj['path']