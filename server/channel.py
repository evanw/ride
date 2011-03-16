import io
import json
import threading

debug = True

class _Server(io.Server, threading.Thread):
    def __init__(self):
        io.Server.__init__(self)
        threading.Thread.__init__(self)
        self.map = {}

    def on_message(self, client, data):
        if debug:
            print '--- received ----'
            print data
        data = json.loads(data)
        
        # broadcast messages received from one client to the others
        for session in self.clients:
            if session != client.session:
                self.clients[session].send(json.dumps(data))
        
        # notify all local handlers of the new message
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
        # Send message to everyone except listener_to_ignore
        if channel in self.map:
            for listener in self.map[channel]:
                if listener_to_ignore != listener:
                    listener.on_message(channel, data)
        
        # Send to clients if the message didn't originate from the server
        if listener_to_ignore != self:
            data = json.dumps({ 'channel': channel, 'data': data })
            self.broadcast(data)
            if debug:
                print '--- sent ----'
                print data

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
