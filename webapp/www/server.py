from SimpleHTTPServer import SimpleHTTPRequestHandler
from SocketServer import TCPServer
import os

print 'http://localhost:8000/'

class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.path = '/static/main/index.html'
        elif self.path.startswith('/project/'):
            self.path = '/static/project/index.html'
        SimpleHTTPRequestHandler.do_GET(self)

class Server(TCPServer):
    allow_reuse_address = True

Server(('', 8000), Handler).serve_forever()
