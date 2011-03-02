'''
Channel module

This module adds channel support to python:

    from channel import subscribe, Channel

    @subscribe('foo', 'bar')
    def foo_bar(data):
        print data, 'sent to channel foo.bar'

    Channel('sample', 'channel', 'name').publish({
        'testing': True,
        'data': None
    })

A channel is named by a list of strings, which was chosen for ease of
construction and parsing. Importing this module automatically starts a thread
that notifies all subscribers of new messages.
'''

import json
import socket
import threading

_lock = threading.Lock()

class _Container:
    def __init__(self):
        self.map = {}
        self.callbacks = []

    def lookup(self, names, index=0):
        if index >= len(names):
            return self
        name = names[index]
        if name not in self.map:
            self.map[name] = _Container()
        return self.map[name].lookup(names, index + 1)

    def subscribe(self, callback):
        self.callbacks.append(callback)

    def unsubscribe(self, callback):
        while callback in self.callbacks:
            self.callbacks.remove(callback)

    def publish(self, data):
        for callback in self.callbacks:
            callback(data)

class Channel:
    def __init__(self, *name):
        self.name = name
    
    def subscribe(self, callback):
        '''Add callback to the list of callbacks for this channel.'''
        _lock.acquire()
        channel = _root.lookup(self.name).subscribe(callback)
        _lock.release()
    
    def unsubscribe(self, callback):
        '''Remove callback from the list of callbacks for this channel.'''
        _lock.acquire()
        channel = _root.lookup(self.name).unsubscribe(callback)
        _lock.release()
    
    def publish(self, data):
        '''
        Send data on this channel.
        
        Note: this will not notify any callbacks for this channel, callbacks
        for this channel will only be notified when data is received.
        '''
        data['channel'] = self.name
        _socket.sendto(json.dumps(data), ('localhost', 5001))

_root = _Container()
_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_socket.bind(('localhost', 5002))

def _run():
    while 1:
        data = json.loads(_socket.recvfrom(2048)[0])
        _lock.acquire()
        _root.lookup(data['channel']).publish(data)
        _lock.release()

_thread = threading.Thread(target=_run)
_thread.daemon = True
_thread.start()

def subscribe(*name):
    '''
    A decorator that makes subscribing to channels easy:
    
        @subscribe('foo', 'bar')
        def foo_bar(data):
            print data, 'sent to channel foo.bar'

    This is equivalent to:
    
        def foo_bar(data):
            print data, 'sent to channel foo.bar'
        
        Channel('foo', 'bar').subscribe(foo_bar)
    '''
    def _channel(func):
        Channel(*name).subscribe(func)
        return func
    return _channel
