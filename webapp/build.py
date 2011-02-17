#!/usr/bin/python

input_path = 'src/'
output_path = 'www/script.js'

import os, sys, time

def compile(sources):
    return '\n'.join(open(f).read() for f in sources)

def sources():
    return [os.path.join(base, f) for base, folders, files in \
        os.walk(input_path) for f in files if f.endswith('.js')]

def build():
    data = compile(sources())
    print 'built %s (%u lines)' % (output_path, len(data.split('\n')))
    open(output_path, 'w').write(data)

def stat():
    return [os.stat(f).st_mtime for f in sources()]

def monitor():
    a = stat()
    while True:
        time.sleep(0.5)
        b = stat()
        if a != b:
            a = b
            build()

if __name__ == '__main__':
    build()
    if 'release' not in sys.argv:
        monitor()
