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

    def run(self):
        a = stat(self.input_path)
        while True:
            time.sleep(0.5)
            b = stat(self.input_path)
            if a != b:
                a = b
                self.build()

if __name__ == '__main__':
    builders = [
        Builder('src/main/', 'www/static/main/generated.js'),
        Builder('src/project/', 'www/static/project/generated.js'),
        Builder('src/codeview/', 'www/static/codeview/generated.js'),
        Builder('src/nodeview/', 'www/static/nodeview/generated.js'),
    ]

    for b in builders:
        b.build()

    if 'release' not in sys.argv:
        for b in builders:
            b.start()

        # wait until Ctrl+C
        try:
            time.sleep(99999999)
        finally:
            sys.exit()
