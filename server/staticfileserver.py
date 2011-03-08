import os
import urllib
import posixpath
from SocketServer import TCPServer
from SimpleHTTPServer import SimpleHTTPRequestHandler

class _Handler(SimpleHTTPRequestHandler):
    HTTP_DIR = '../webapp/www'

    def do_GET(self):
        if self.path == '/':
            self.path = '/static/main/index.html'
        elif self.path.startswith('/project/'):
            self.path = '/static/project/index.html'
        SimpleHTTPRequestHandler.do_GET(self)

    # overridden to specify custom path, otherwise would use current directory
    # copied and pasted from source code for SimpleHTTPRequestHandler
    def translate_path(self, path):
        """Translate a /-separated PATH to the local filename syntax.

        Components that mean special things to the local file system
        (e.g. drive or directory names) are ignored.  (XXX They should
        probably be diagnosed.)
       
        """ 
        # abandon query parameters 
        path = path.split('?',1)[0]
        path = path.split('#',1)[0]
        path = posixpath.normpath(urllib.unquote(path))
        words = path.split('/')
        words = filter(None, words)
        path = _Handler.HTTP_DIR # os.getcwd()
        for word in words:
            drive, word = os.path.splitdrive(word)
            head, word = os.path.split(word)
            if word in (os.curdir, os.pardir): continue
            path = os.path.join(path, word)
        return path

class _WebServer(TCPServer):
    allow_reuse_address = True

def serve_forever(port):
    print 'serving website on http://localhost:%d/' % port
    _WebServer(('', port), _Handler).serve_forever()
