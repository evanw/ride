import io
import json
import threading

class _Server(io.Server, threading.Thread):
    def __init__(self):
        io.Server.__init__(self)
        threading.Thread.__init__(self)
        self.map = {}

    def on_message(self, client, data):
        data = json.loads(data)
        self.publish(tuple(data['channel']), data['data'], self)

    def subscribe(self, listener, channel):
        if not channel in self.map:
            self.map[channel] = []
        self.map[channel].append(listener)

    def unsubscribe(self, listener, channel):
        if channel in self.map:
            self.map[channel].remove(listener)
            if not self.map[channel]:
                del self.map[channel]

    def publish(self, channel, data, listener_to_ignore):
        if channel in self.map:
            for listener in self.map[channel]:
                if listener_to_ignore != listener:
                    listener.on_message(channel, data)
        if listener_to_ignore != self:
            self.broadcast(json.dumps({ 'channel': channel, 'data': data }))

_server = _Server()

class Listener:
    def on_message(self, channel, message):
        pass

    def subscribe(self, *channel):
        _server.subscribe(self, tuple(map(str, channel)))

    def unsubscribe(self, *channel):
        _server.unsubscribe(self, tuple(map(str, channel)))

    def publish(self, channel, data):
        _server.publish(tuple(map(str, channel)), data, self)

def _run():
    _server.listen(5000)

_thread = threading.Thread(target=_run)
_thread.daemon = True
_thread.start()
