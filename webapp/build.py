#!/usr/bin/python

import os, sys, time
from threading import Thread

def compile(sources):
    return '\n'.join(open(f).read() for f in sources)

def sources(path):
    return [os.path.join(base, f) for base, folders, files in \
        os.walk(path) for f in files if f.endswith('.js')]

def stat(path):
    return [os.stat(f).st_mtime for f in sources(path)]

class Builder(Thread):
    def __init__(self, input_path, output_path):
        Thread.__init__(self)
        self.input_path = input_path
        self.output_path = output_path

    def build(self):
        data = compile(sources(self.input_path))
        open(self.output_path, 'w').write(data)
        print 'built %s (%u lines)' % (self.output_path, len(data.split('\n')))

    def monitor(self):
        a = stat(self.input_path)
        while True:
            time.sleep(0.5)
            b = stat(self.input_path)
            if a != b:
                a = b
                self.build()

    def run(self):
        self.build()
        if 'release' in sys.argv:
            self.monitor()

if __name__ == '__main__':
    Builder('src/main_page/', 'www/generated.js').start()
    Builder('src/node_editor/', 'www/node_editor/generated.js').start()

    while True:
        time.sleep(1)
