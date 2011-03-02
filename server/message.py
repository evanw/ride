import json

class Message:
    def __init__(self, data):
        obj = json.load(data)
        self.action = obj.action
        self.path = obj.path