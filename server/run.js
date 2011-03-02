/*
This is a small shim to get socket.io in python.

In python, create a UDP socket on port 5002 and send and receive messages to
and from localhost on port 5001.

In the browser, include http://localhost:5000/socket.io/socket.io.js and
connect to port 5000.
*/

var fs = require('fs');
var http = require('http');
var dgram = require('dgram');
var io = require('socket.io');

// http server
var server = http.createServer(function(req, res) {
	res.writeHead(200, { 'Content-Type': 'text/plain' });
	res.end('go away');
});
server.listen(5000);

// udp connection to server
var udp = dgram.createSocket('udp4');
udp.on('message', function(data) {
	data = data.toString();
	for (var i = 0; i < socket.clients.length; i++) {
		socket.clients.send(data);
	}
});
udp.bind(5001, 'localhost');

// socket.io connection to clients
var socket = io.listen(server);
socket.on('connection', function(client) {
	client.on('message', function(data) {
		udp.send(new Buffer(data), 0, data.length, 5002, 'localhost');
	});
});
