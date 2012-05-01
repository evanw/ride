#!/usr/bin/python
from ROSProxy import ROSProxy
from terribleWSS import Session, serveForever
import sys, traceback
import json
from threading import Lock, Thread
from time import time, sleep
from urllib2 import urlopen
import re

class ServiceCaller(Thread):
	def __init__(self, ros, session, service, receiverID, arguments):
		Thread.__init__(self)
		self.daemon = True
		self.ros = ros
		self.session = session
		self.service = service
		self.receiverID = receiverID
		self.arguments = arguments

	def run(self):
		try:
			def handleResponse(rsp):
				call = {'receiver':self.service + self.receiverID,'msg':self.ros.generalize(rsp)}
				self.session.sock.send(encode(call))
			self.ros.callService(str(self.service),self.arguments,handleResponse)
		except:
			print "Problem calling service:"
			traceback.print_exc()

def encode(obj):
	return '\x00' + json.dumps(obj) + '\n' + '\xff'

def handleFrameHelper(frame, session, handleMsgFactory, sub, passfile, ros):
	try:
		call = ''
		if frame[0] != '\x00':
			call = json.loads(frame)
		else:
			call = json.loads(frame[1:])

		orgReceiver = call["receiver"]
		receiver = re.sub(r'^\/rosjs','/rosbridge',call["receiver"])
		msg = call["msg"]


		if 'type' in call.keys():
			#print "Publish Topic!"
			if (not passfile or (session.authStatus == 'accepted')):
				typ = call["type"]

				first = True
				if not receiver in session.data.keys():
					session.publishers[receiver] = rospy.Publisher(receiver.encode('ascii','ignore'), ros.msgClassFromTypeString(typ))
				else:
					first = False

				if msg != {} and msg != [] or not first:
					session.publishers[receiver].publish(ros.specify(typ,msg))

		else:
			#print "Service Call!"

			#prep the receiverID
			receiverParts = receiver.split('#')
			receiverID = ('#').join(receiverParts[1:])
			if receiverID != '':
				receiverID = '#' + receiverID
			receiver = receiverParts[0]

			if (not passfile or (session.authStatus == 'accepted') or receiver.find('/rosbridge/challenge') == 0 or receiver.find('/rosbridge/authenticate') == 0):
				if receiver.find('/rosbridge') == 0:
					if (receiver == "/rosbridge/topics"):
						call = {'receiver':orgReceiver,'msg':ros.topics}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/publish"):
						receiver = msg[0].encode('ascii','ignore')
						typ = msg[1]
						if not receiver in session.data.keys():
							session.data[receiver] = rospy.Publisher(receiver.encode('ascii','ignore'), ros.msgClassFromTypeString(typ))
						call = {'receiver':orgReceiver,'msg':'OK'}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/services"):
						call = {'receiver':orgReceiver,'msg':ros.services}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/get_param"):
						name = msg[0].encode('ascii','ignore')
						value = None
						try:
							if len(msg) == 2:
								value = msg[1]
								value = rospy.get_param(name, value)
							else:
								value = rospy.get_param(name)
						except KeyError:
							pass
						call = {'receiver':orgReceiver,'msg':value}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/set_param"):
						name = msg[0].encode('ascii','ignore')
						value = msg[1]
						rospy.set_param(name, value)
						call = {'receiver':orgReceiver,'msg':None}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/has_param"):
						name = msg[0].encode('ascii','ignore')
						value = rospy.has_param(name)
						call = {'receiver':orgReceiver,'msg':value}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/delete_param"):
						name = msg[0].encode('ascii','ignore')
						value = None
						try:
							value = rospy.get_param(name)
							rospy.delete_param(name)
						except KeyError:
							pass
						call = {'receiver':orgReceiver,'msg':value}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/search_param"):
						name = msg[0].encode('ascii','ignore')
						params = rospy.search_param(name)
						call = {'receiver':orgReceiver,'msg':params}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/get_param_names"):
						params = rospy.get_param_names()
						call = {'receiver':orgReceiver,'msg':params}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/unsubscribe"):
						topic = msg[0].encode('ascii','ignore')
						print "unsubscribing to: %s" % (topic,)
						session.latestLock.acquire()
						if topic in session.latest.keys():
							del session.latest[topic]
						if topic in session.latestQueue.keys():
							del session.latestQueue[topic]
						if topic in session.latestSubs.keys():
							session.latestSubs[topic].unregister()
							del session.latestSubs[topic]
						session.latestLock.release()
						call = {'receiver':orgReceiver,'msg':'OK'}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/subscribe"):
						topic = msg[0].encode('ascii','ignore')
						delay = msg[1]
						drop = False if delay == -1 else True # Whether to have an infinte queue or not
						status = 'OK'
						handleLatch = None

						if len(msg) == 3:
							status,handleLatch = sub(session, topic, handleMsgFactory(session, topic), typ=msg[2], drop=drop)
						else:
							status,handleLatch = sub(session, topic, handleMsgFactory(session, topic), typ=None, drop=drop)
						session.data[topic] = {'delay':delay,'lastEmission':0,'lastSeq':0}

						if delay == -1:
							session.latestLock.acquire()
							if not topic in session.latestQueue.keys():
								session.latestQueue[topic] = []
							session.latestLock.release()

						call = {'receiver':orgReceiver,'msg':status}
						session.sock.send(encode(call))

						if handleLatch:
							handleLatch()

					elif (receiver == "/rosbridge/log"):
						filename = msg[0].encode('ascii','ignore')
						filename = ''.join(c for c in filename if c.isalnum()) + '.log'
						obj = msg[1];

						success = True
						try:
							log = open(filename, 'w')
							log.write(obj)
							log.close()
						except:
							success = False

						call = {'receiver':orgReceiver,'msg':'OK'}
						if (not success):
							call = {'receiver':orgReceiver,'msg':'ERROR'}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/challenge"):
						user = msg[0].encode('ascii','ignore')

						call = {'receiver':orgReceiver,'msg':{'challenge':'','salt':''}}
						if not user in passfile.keys():
							session.authStatus = 'rejected'
						else:
							session.challenge = gensalt(5)
							session.user = user
							call = {'receiver':orgReceiver,'msg':{'challenge':session.challenge,'salt':passfile[user]['salt']}}
							session.sock.send(encode(call))
					elif (receiver == "/rosbridge/authenticate"):
						response = msg[0].encode('ascii','ignore')
						salt = msg[1].encode('ascii','ignore')

						if (response == hashpw(passfile[session.user]['hash']+session.challenge,salt)):
							session.authStatus = 'accepted'
							call = {'receiver':orgReceiver,'msg':session.authStatus}
							session.sock.send(encode(call))
						else:
							session.authStatus = 'rejected'
					elif (receiver == "/rosbridge/typeStringFromTopic"):
						topic = msg[0].encode('ascii','ignore')
						call = {'receiver':orgReceiver,'msg':ros.typeStringFromTopic(topic)}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/typeStringFromService"):
						service = msg[0].encode('ascii','ignore');
						call = {'receiver':orgReceiver,'msg':ros.typeStringFromService(service)}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/msgClassFromTypeString"):
						typStr = msg[0].encode('ascii','ignore');
						call = {'receiver':orgReceiver,'msg':ros.generalize(ros.msgClassFromTypeString(typStr)())}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/reqClassFromTypeString"):
						typStr = msg[0].encode('ascii','ignore');
						call = {'receiver':orgReceiver,'msg':ros.generalize(ros.srvClassFromTypeString(typStr)._request_class())}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/rspClassFromTypeString"):
						typStr = msg[0].encode('ascii','ignore');
						call = {'receiver':orgReceiver,'msg':ros.generalize(ros.srvClassFromTypeString(typStr)._response_class())}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/classFromTopic"):
						topic = msg[0].encode('ascii','ignore')
						call = {'receiver':orgReceiver,'msg':ros.generalize(ros.classFromTopic(topic)())}
						session.sock.send(encode(call))
					elif (receiver == "/rosbridge/classesFromService"):
						service = msg[0].encode('ascii','ignore')
						call = {'receiver':orgReceiver,'msg':{'req':ros.generalize(ros.classFromService(service)._request_class()),'rsp':ros.generalize(ros.classFromService(service)._response_class())}}
						session.sock.send(encode(call))
				else:
					idx = receiver.find('protected')
					if idx >= 0 and idx <=1:
						print "ignoring call to protected service"
						call = {'receiver':orgReceiver,'msg':{}}
						session.sock.send(encode(call))
					else:
						cls = ros.classFromService(receiver)
						arg = None;
						if isinstance(msg, list):
							arg = ros.specify(cls._request_class._slot_types, msg)
						else:
							arg = ros.specify(cls._request_class, msg)
						serviceCaller = ServiceCaller(ros,session,receiver,receiverID,arg)
						serviceCaller.start()

	except:
		print "Problem " * 10
		traceback.print_exc()
		print "Problem " * 10

	if (not passfile or session.authStatus != 'rejected'):
		session.transition(Session.ready)
	else:
		session.transition(Session.closed)

def handleFrameFactory(handleMsgFactory, sub, passfile, ros):
	def handleFrame(frame, session):
		return handleFrameHelper(frame, session, handleMsgFactory, sub, passfile, ros)
	return handleFrame

if __name__ == "__main__":
	ros = ROSProxy()
	import rospy
	rospy.init_node('rosbridge')

	passfile = rospy.get_param('/brown/rosbridge/passfile','')
	if "--passfile" in sys.argv:
		idx = sys.argv.index("--passfile")+1
		if idx < len(sys.argv):
			passfile = sys.argv[idx]
		else:
			print "Please provide the path to the passfile."
			sys.exit(-1)
	if (passfile != ''):
		print "Only users from the passfile will be accepted."
		filesrc = open(passfile,'r')	
		passfile = filesrc.readlines()
		passfile = ''.join(passfile)
		passfile = json.loads(passfile)
		filesrc.close()
		from bcrypt import hashpw, gensalt

	host = rospy.get_param('/brown/rosbridge/host','')
	if "--host" in sys.argv:
		idx = sys.argv.index("--host")+1
		if idx < len(sys.argv):
			host = sys.argv[idx]
			print "rosbridge is bound to %s." % (host,)
		else:
			print "Please provide a hostname."
			sys.exit(-1)

	port = rospy.get_param('/brown/rosbridge/port',9090)
	if "--port" in sys.argv:
		idx = sys.argv.index("--port")+1
		if idx < len(sys.argv):
			port = int(sys.argv[idx])
			print "rosbridge is will use port %s." % (port,)
		else:
			print "Please provide a port number."
			sys.exit(-1)

	hz = rospy.get_param('/brown/rosbridge/hz',100000)
	if "--hz" in sys.argv:
		idx = sys.argv.index("--hz")+1
		if idx < len(sys.argv):
			hz = int(sys.argv[idx])
			print "rosbridge will sample at %shz." % (hz,)
		else:
			print "Please provide a sample rate (in hz)."
			sys.exit(-1)

	def sub(session, topic, handler, typ, drop):
		idx = topic.find('protected')
		if idx >= 0 and idx <=1:
			print "ignoring request to listen to protected topic"
			return
		session.latestLock.acquire()
		repeat = False
		if topic in session.latestSubs.keys():
			repeat = True
		session.latestLock.release()

		status = 'OK'
		latch = None
		subscriber = None

		if not repeat:
			try:
				print "subscribing to: %s" % (topic,)
				cls = None
				if typ == None:
					cls = ros.classFromTopic(topic)
				else:
					cls = ros.msgClassFromTypeString(typ)
				subscriber = rospy.Subscriber(topic, cls, handler, queue_size=10 if drop else None)
				session.latestLock.acquire()
				session.latestSubs[topic] = subscriber
				session.latestLock.release()
			except:
				print "subscription to %s failed." % (topic,)
				status = 'ERROR'
		else:
			session.latestLock.acquire()
			subscriber = session.latestSubs[topic]
			session.latestLock.release()

		if len(subscriber.impl.connections):
			latch = subscriber.impl.connections[-1].latch
                        if latch:
                                def handleLatchFactory(handle,latch):
                                        def handleLatch():
                                                handler(latch)
                                        return handleLatch
                                latch = handleLatchFactory(handler,latch)

		return status,latch

	def handleMsgFactory(session, topic):
		def handleMsg(msg):
			session.latestLock.acquire()

			seq = 0
			if topic in session.latest.keys():
				seq = session.latest[topic]['seq']
			item= {'seq':seq + 1,'msg':msg}
			session.latest[topic] = item
			if topic in session.latestQueue.keys():
				session.latestQueue[topic].append(item)

			session.latestLock.release()

		return handleMsg

	def handleOutput(session):
		if (passfile):
			if (session.authStatus == 'rejected'):
				session.transition(Session.closed)
				return

		for topic in session.data.keys():
			delay = session.data[topic]['delay']
			lastEmission = session.data[topic]['lastEmission']
			lastSeq = session.data[topic]['lastSeq']
			data = session.data[topic]

			session.latestLock.acquire()

			if topic in session.latest.keys():
				current = session.latest[topic]

				#release as soon as possible
				session.latestLock.release()

				if current['seq'] > lastSeq or delay == -1:
					elapsed = (time() - lastEmission)*1000

					if delay == -1:
						if len(session.latestQueue[topic]) == 0:
							current = None
						else:
							current = session.latestQueue[topic].pop(0)

					if elapsed > delay and current != None:
						call = {'receiver':topic, 'msg': ros.generalize(current['msg'])}
						session.sock.send(encode(call))
						session.data[topic] = {'delay':delay,'lastEmission':time(),'lastSeq':current['seq']}
			else:
				session.latestLock.release()

	def loop():
		return not rospy.is_shutdown()

	restart = True
	while loop():
		if restart:
			restart = False
			try:
				serveForever(handleFrameFactory(handleMsgFactory, sub, passfile, ros), handleOutput, loop, host, port, 1.0/(hz*2.0))
			except:
				restart = True
				print "Problem " * 10
				traceback.print_exc()
				print "Problem " * 10
		sleep(1)
