#!/usr/bin/env python

import sqlite3
import socket
import threading
import daemon

# TODO: Daemonize this.

class BasicSearchThread(threading.Thread):
    def __init__(self, channel, details):
        self.channel = channel
        self.details = details
        threading.Thread.__init__(self)
        print('init...')
    
    def run(self):
        print('Connection received from ' + self.details[0] + '.')
        db = sqlite3.connect('rosrepos.sqlite')
        c = db.cursor()
        cols = {'d':'desc', 'r':'repo', 'p':'pkg', 'sd':'shortdesc', 'l':'license', \
                's':'source','n':'deps','w':'website'}
        
        while True:
            # Get search string from client.
            data = self.channel.recv(1024).strip()
            if data == '.':
                break
            
            # Split the search string into its components.
            pieces = data.split(',')
            for i in range(len(pieces)):
                pieces[i] = pieces[i].strip()
            request = [p.split(':') for p in pieces]
            
            # Generate the SQL query from the search string.
            sql = 'select pkg, repo, shortdesc from packages where '
            for r in request:
                sql += '%s like "%%%s%%" and ' % ((cols[r[0]], r[1]) if len(r) > 1 else ('pkg', r[0]))
            sql = sql[:-5]
            
            # Perform the query, and format the results for output.
            out = ''
            for e in c.execute(sql).fetchall():
                out += str(e) + '\n'
            self.channel.send(out)

        c.close()
        db.close()        
        self.channel.close()

# A simple server to farm requests out to handler threads.
class SearchServer():
    def __init__(self):
        # Create a standard, TCP socket, and bind it to port 7433.
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((socket.gethostname(), 7434))
        self.server.listen(5)
        
    def start(self):
        # Listen for connections and handle them in separate threads.
        while True:
            channel, details = self.server.accept()
            BasicSearchThread(channel, details).start()

if __name__ == '__main__':
    SearchServer().start()

