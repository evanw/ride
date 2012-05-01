#!/usr/bin/python
import socket
from select import select
from threading import Lock, Thread
from struct import pack, unpack
import re
import hashlib
from base64 import b64encode

class Sender(Thread):
	def __init__(self,proxy):
		Thread.__init__(self)
		self.proxy = proxy
		self.go = True
		self.latestException = None
		self.lock = Lock()
		self.lock.acquire()

	def run(self):
		while self.go:
			self.lock.acquire()
			if (not self.go):
				self.proxy.lock.release()
				break
			msg = self.proxy.msg
			args = self.proxy.args
			sent = 0
			toSend = len(msg)
			while sent < toSend:
				justSend = 0
				try:
					justSent = self.proxy.sock.send(msg[sent:],*args)
				except Exception, e:
					justSent = 0
					self.latestException = e
				sent = sent + justSent
			self.proxy.lock.release()

class SockProxy(object):
	def __init__(self,sock):
		self.raw = True
		self.sock = sock
		self.lock = Lock()
		thisSelf = self
		self.msg = ''
		self.args = []
		self.sender = Sender(thisSelf)
		self.sender.start()

	def send(self,msg,*args):
		if not self.raw:
			#correct the msg
			length = len(msg)-2
			if length < 126:
				msg = '\x81' + pack('!B',length) + msg[1:-1]
			elif length <= 65535:
				msg = '\x81' + pack('!B',126) + pack('!H',length) + msg[1:-1]
			else:
				msg = '\x81' + pack('!B',127) + pack('!Q',length) + msg[1:-1]
		if self.lock.acquire(True):
			self.msg = msg
			self.args = args
			self.sender.lock.release()

	def close(self):
		self.lock.acquire()
		self.sender.go = False
		self.sender.lock.release()

class Session(object):
	handshake = 0
	determineResponse = 1
	ready = 2
	receiveKey = 3
	sentinel = 4
	closed = 5
	finAndOpCode = 6
	sixteenBitLength = 7
	sixtyFourBitLength = 8
	maskingKey = 9
	payload = 10
	def __init__(self,sock):
		self.sock = SockProxy(sock)
		self.buffer = []
		self.state = Session.handshake
		self.count = 0
		self.data = {}
		self.msg = []
		self.opcode = 0
		self.fin = 0
		self.mask = 0
		self.length = 0
		self.publishers = {}
		self.authStatus = ''
		self.latest = {}
		self.latestQueue = {}
		self.latestSubs = {}
		self.latestLock = Lock()

	def transition(self,state,flush=False):
		self.count = 0
		self.state = state
		if flush:
			self.buffer = []

def hybiHandshake(buffer):
	resp = ["HTTP/1.1 101 Switching Protocols\r\n"]
	resp.append("Upgrade: websocket\r\n")
	resp.append("Connection: Upgrade\r\n")

	key = re.compile(r'.*Sec-WebSocket-Key:\s*(.*?)[\r\n]',re.DOTALL).match(buffer).group(1)
	magicString = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11" #from the draft protocol

	resp.append("Sec-WebSocket-Accept: ")
	resp.append(b64encode(hashlib.sha1(key + magicString).digest()))
	resp.append("\r\n")

	resp.append("\r\n")

	return ''.join(resp)

def googleHandshake(buffer):
	resp = ["HTTP/1.1 101 Web Socket Protocol Handshake\r\n"]
	resp.append("Upgrade: WebSocket\r\n")
	resp.append("Connection: Upgrade\r\n")

	resp.append("WebSocket-Location: ws://")
	resp.append(re.compile(r'.*Host:\s*([^\r\n]*).*',re.DOTALL).match(buffer).group(1))
	resp.append("/\r\n")

	resp.append("WebSocket-Origin: ")
	resp.append(re.compile(r'.*Origin:\s*([^\r\n]*).*',re.DOTALL).match(buffer).group(1))
	resp.append("\r\n")

	resp.append("\r\n")

	return ''.join(resp)

def actualHandshake(buffer):
	try:
		keyNumber1 = re.compile(r'.*Sec-WebSocket-Key1: ([^\r\n]*)\s*.*',re.DOTALL).match(buffer).group(1)
		spaces1 = len(re.findall(' ',keyNumber1))
		keyNumber1 = re.sub(r'[^0123456789]','',keyNumber1)
		keyNumber1 = int(keyNumber1)
		keyNumber2 = re.compile(r'.*Sec-WebSocket-Key2: ([^\r\n]*)\s*.*',re.DOTALL).match(buffer).group(1)
		spaces2 = len(re.findall(' ',keyNumber2))
		keyNumber2 = re.sub(r'[^0123456789]','',keyNumber2)
		keyNumber2 = int(keyNumber2)


		if (spaces1 == 0 or spaces2 == 0):
			return None

		if (keyNumber1 % spaces1 != 0 or keyNumber2 % spaces2 != 0):
			return None

		part1 = keyNumber1 / spaces1
		part2 = keyNumber2 / spaces2

		challenge = pack('!ii',part1,part2) + buffer[-8:]

		checker = hashlib.md5()
		checker.update(challenge)

		response = checker.digest()

		resp = ["HTTP/1.1 101 Web Socket Protocol Handshake\r\n"]
		resp.append("Upgrade: WebSocket\r\n")
		resp.append("Connection: Upgrade\r\n")

		resp.append("Sec-WebSocket-Location: ws://")
		resp.append(re.compile(r'.*Host:\s*([^\r\n]*).*',re.DOTALL).match(buffer).group(1))
		resp.append("/\r\n")

		resp.append("Sec-WebSocket-Origin: ")
		resp.append(re.compile(r'.*Origin:\s*([^\r\n]*).*',re.DOTALL).match(buffer).group(1))
		resp.append("\r\n")

		resp.append("\r\n")

		resp.append(response)

		return ''.join(resp)

	except:
		return None

def defaultHandleFrame(frame, session):
	session.transition(Session.ready)

def defaultHandleOutput(session):
	pass

def defaultLoop():
	pass

def serveForever(handleFrame = defaultHandleFrame, handleOutput=defaultHandleOutput, loop=defaultLoop, host='', port=9090, hz=1.0/100000):
	incoming = []
	outgoing = []
	try:
		serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		serverSocket.bind((host,port))
		serverSocket.listen(5)

		incoming = [serverSocket]
		sessions = {}

		def closeSocket(sock):
			print "closed %s" % (sock.fileno(),)
			if sock.fileno() in sessions.keys():
				sessions[sock.fileno()].sock.close()
				del sessions[sock.fileno()]
			incoming.remove(sock)
			outgoing.remove(sock)
			try:
				sock.shutdown(socket.SHUT_RDWR)
				sock.close()
			except:
				print "Socket did not close smoothly."

		def closeSockets():
			for i in sessions.keys():
				session = sessions[i]
				if session.state == Session.closed:
					closeSocket(session.sock.sock)

		while loop():
			inputReady,outputReady,errors = select(incoming,[],[],hz)

			for input in inputReady:
				if (input == serverSocket):
					#new connection
					connection, address = serverSocket.accept()
					print "Connection from %s:%s" % address
					sessions[connection.fileno()] = Session(connection)
					incoming.append(connection)
					outgoing.append(connection)

					print "%s concurrent connections.\n" % (len(incoming),)

				else:
					session = sessions[input.fileno()]
					try:
						buff = input.recv(8192)
					except socket.error:
						# Reset from remote client.
						closeSocket(input)
						continue

					buffLength = len(buff)
					if buffLength == 0:
						closeSocket(input)
						continue

					idx = -1
					while True:
						idx = idx + 1
						if idx >= buffLength:
							break

						data = buff[idx]
						session.buffer.append(data)

						if (session.state == Session.handshake):
							if "\r\n\r\n" ==  ''.join(session.buffer[-4:]):
								session.transition(Session.determineResponse)

						if (session.state == Session.determineResponse):
							call = ''.join(session.buffer[:-1])

							print "------"
							print call
							print "------"

							if re.compile(r'.*Sec-WebSocket-Key2',re.DOTALL).match(call) != None:
								print "\"original\" websocket handshake (will be deprecated)"
								session.transition(Session.receiveKey)

							elif re.compile(r'raw').match(call) != None:
								print "raw socket"
								session.transition(Session.ready)
								
							elif re.compile(r'.*Sec-WebSocket-Key[^0-9]',re.DOTALL).match(call) != None:
								print "draft-ietf-hybi-thewebsocketprotocol-06 (preliminary, _may_ be deprecated)"
								input.send(hybiHandshake(call))
								session.sock.raw = False
								session.transition(Session.finAndOpCode)
								continue

							else:
								#google handshake
								print "Google handshake (will be deprecated)"
								input.send(googleHandshake(call))
								session.transition(Session.ready)

							print "------\n"

						if (session.state == Session.receiveKey):
							if (session.count >= 8):
								resp = actualHandshake(''.join(session.buffer))
								if resp != None:
									input.send(resp)
									session.transition(Session.ready)
									continue
								else:
									closeSocket(input)
							else:	
								session.count = session.count + 1

						if (session.state == Session.ready):
							data = unpack('!b',data)[0]
							if (data >> 7) == 0:
								session.transition(Session.sentinel, flush=True)
								continue
							else:
								print "Binary data frames are unsupported"
								closeSocket(input)

						if (session.state == Session.sentinel):
							found = buff.find('\xff',idx)
							if found != -1:
								session.buffer.extend(buff[idx+1:found])
								frame = ''.join(session.buffer)
								handleFrame(frame, session)
								idx = found
							else:
								session.buffer.extend(buff[idx+1:])
								idx = buffLength-1

						if (session.state == Session.finAndOpCode):
							session.count = session.count + 1
							if session.count == 1:
								data = unpack('!B',data)[0]
								fin = data >> 7
								opcode = data & 15
								session.fin = fin
								session.opcode = opcode
							if session.count == 2:
								data = unpack('!B',data)[0]
								mask = data >> 7
								if mask != 1:
									print "Only masked frames are supported."
									closeSocket(input)
								length = data & 127
								session.length = length
								if session.length == 126:
									session.transition(Session.sixteenBitLength, flush=True)
									continue
								if session.length == 127:
									session.transition(Session.sixtyFourBitLength, flush=True)
									continue
								session.transition(Session.maskingKey, flush=True)
								continue

						if (session.state == Session.sixteenBitLength):
							session.count = session.count + 1
							if session.count == 2:
								length = ''.join(session.buffer)
								session.length = unpack('!H',length)[0]
								session.transition(Session.maskingKey, flush=True)
								continue

						if (session.state == Session.sixtyFourBitLength):
							session.count = session.count + 1
							if session.count == 8:
								length = ''.join(session.buffer)
								session.length = unpack('!Q',length)[0]
								session.transition(Session.maskingKey, flush=True)
								continue

						if (session.state == Session.maskingKey):
							session.count = session.count + 1
							if session.count == 4:
								session.mask = session.buffer + []
								session.transition(Session.payload, flush=True)
								continue

						if (session.state == Session.payload):
							more = -len(session.buffer)
							span = idx+(session.length-session.count)
							session.buffer.extend(buff[idx+1:span])
							more = more + len(session.buffer)

							session.count = session.count + more + 1
							idx = idx + more

							if session.count == session.length:
								msg = session.buffer + []
								mask = session.mask
								for i in xrange(session.length):
									msg[i] = chr(ord(msg[i]) ^ ord(mask[i % 4]))
								if session.opcode == 1 or session.opcode == 0:
									session.msg = session.msg + msg
								if session.fin == 1 and session.opcode == 1 or session.opcode == 0:
									handleFrame(''.join(session.msg),session)
									session.msg = []
								if session.opcode == 8:
									closeSocket(input)
								if session.opcode == 9:
									input.send('\x89' + '\x00')
								session.buffer = list(buff[span:])
								session.transition(Session.finAndOpCode)



			inputReady,outputReady,errors = select([],outgoing,[],hz)

			for output in outputReady:
				try:	
					handleOutput(sessions[output.fileno()])
				except socket.error, e:
					pass

			closeSockets()

	except:
		raise

	finally:
		socks = incoming + filter(lambda x:x not in incoming,outgoing)
		for sock in socks:
			try:
				if sock.fileno() in sessions.keys():
					sessions[sock.fileno()].sock.close()
				print "closing sock: %s" % (sock.fileno(),)
				sock.shutdown(1)
				sock.close()
			except:
				print "failed to close sock: %s" % (sock.fileno(),)
