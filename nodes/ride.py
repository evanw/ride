#!/usr/bin/env python
import roslib; roslib.load_manifest('ride')

import re
import os
import sys
import rospy
import pickle
import signal
import ride.msg
import ride.srv
import tempfile
import subprocess
from ros import rosgraph
import rosgraph.masterapi
import xml.etree.ElementTree as etree

def run(*args):
	'''run the provided command and return its stdout'''
	print ' '.join(args)
	return subprocess.check_output(args).strip()

def split_words(text):
	'''return a list of lines where each line is a list of words'''
	return [line.strip().split() for line in text.split('\n')]

# Map of node names to Node instances
owned_nodes = {}
node_names = []

class Node:
	def __init__(self, package, binary, name):
		self.package = package
		self.binary = binary
		self.name = name
		self.proc = None
		self.launch = tempfile.mktemp('.launch')

	def __del__(self):
		self.stop()
		os.unlink(self.launch)

	def start(self):
		if self.proc:
			return

		# Create a roslaunch file
		doc = etree.ElementTree(etree.XML('<launch><node/></launch>'))
		node = doc.find('node')
		node.set('name', self.name[1:] if self.name.startswith('/') else self.name)
		node.set('pkg', self.package)
		node.set('type', self.binary)

		# Launch the node
		doc.write(self.launch)
		self.proc = subprocess.Popen(['roslaunch', self.launch])

	def stop(self):
		if self.proc:
			self.proc.send_signal(signal.SIGINT)
			self.proc = None

def unique_name(like):
	like = re.sub('[^A-Za-z0-9]', '_', like)
	names = set(node_names)
	if like not in names:
		return like
	i = 2
	while True:
		name = like + str(i)
		if name not in names:
			return name

def node_create(request):
	'''implements the /ride/node/create service'''
	name = '/' + unique_name(request.binary)
	owned_nodes[name] = Node(request.package, request.binary, name)
	return ride.srv.NodeCreateResponse(request.id, name)

def node_destroy(request):
	'''implements the /ride/node/destroy service'''
	if request.name in owned_nodes:
		del owned_nodes[request.name]
		return ride.srv.NodeDestroyResponse(request.id, True)
	return ride.srv.NodeDestroyResponse(request.id, False)

def node_start(request):
	'''implements the /ride/node/start service'''
	if request.name in owned_nodes:
		owned_nodes[request.name].start()
		return ride.srv.NodeStartResponse(request.id, True)
	return ride.srv.NodeStartResponse(request.id, False)

def node_stop(request):
	'''implements the /ride/node/stop service'''
	if request.name in owned_nodes:
		owned_nodes[request.name].stop()
		return ride.srv.NodeStopResponse(request.id, True)
	return ride.srv.NodeStopResponse(request.id, False)

def node_list(request):
	'''implements the /ride/node/list service'''

	# Query the master for the entire state of the system
	nodes = {}
	publishers, subscribers, services = rosgraph.masterapi.Master('/ride').getSystemState()

	# Link up published topics
	for topic, names in publishers:
		for name in names:
			if name not in nodes:
				nodes[name] = ride.msg.Node(name, [], [], [], False, True)
			nodes[name].published.append(topic)

	# Link up subscribed topics
	for topic, names in subscribers:
		for name in names:
			if name not in nodes:
				nodes[name] = ride.msg.Node(name, [], [], [], False, True)
			nodes[name].subscribed.append(topic)

	# Link up provided services
	for service, names in services:
		for name in names:
			if name not in nodes:
				nodes[name] = ride.msg.Node(name, [], [], [], False, True)
			nodes[name].services.append(service)

	# Make sure all owned nodes are returned and are marked as owned
	for name in owned_nodes:
		if name not in nodes:
			nodes[name] = ride.msg.Node(name, [], [], [], False, False)
		nodes[name].owned = True

	# Save current node names for when we need to generate a new unique name
	node_names = nodes.keys()

	return ride.srv.NodeListResponse(request.id, nodes.values())

def package_list(request):
	'''implements the /ride/package/list service'''

	# Find the names and locations of all packages
	lines = split_words(run('rospack', 'list'))
	packages = [ride.msg.Package(name, path, []) for name, path in lines]

	# Search for binaries using find (this is how rosrun does autocomplete)
	for i, package in enumerate(packages):
		print '[%d/%d]' % (i + 1, len(packages)),
		binaries = run('find', package.path, '-type', 'f', '-perm', '+111')
		if binaries:
			# People make random stuff executable so try to filter that out
			lines = binaries.split('\n')
			extensions = [
                '.cmake',
                '.cpp',
                '.css',
                '.c',
                '.dox',
                '.dsp',
                '.gif',
                '.hpp',
                '.html',
                '.h',
                '.jpg',
                '.log',
                '.make',
                '.msg',
                '.pdf',
                '.png',
                '.sln',
                '.srv',
                '.txt',
                '.vcproj',
                '.xml',
			]
			package.binaries = [line for line in lines if
				all(not line.endswith(ext) for ext in extensions) and
				'/CMakeFiles/' not in line and
				'Makefile' not in line and
				'/build/' not in line and
				'/.svn/' not in line and
				'/.git/' not in line]

	return ride.srv.PackageListResponse(request.id, packages)

def main():
	# Cache the package list because it's expensive to compute
	cache_path = os.path.join(os.path.dirname(__file__), '.package_cache')
	print 'loading all packages...'
	packages = None
	if '--cached' in sys.argv:
		try:
			packages = pickle.load(open(cache_path))
		except:
			pass
	if packages is None:
		packages = package_list(ride.srv.PackageListRequest('')).packages
		try:
			pickle.dump(packages, open(cache_path, 'w'))
		except:
			pass
	print 'finished loading'
	cached_package_list = lambda request: ride.srv.PackageListResponse(request.id, packages)

	# Start up a web server because there's no way to do that from roslaunch.
	# Unfortunately the SimpleHTTPServer API is so simple you can't set the
	# directory other than using the current directory, which we don't want
	# to mess with.
	path = os.path.join(os.path.dirname(__file__), '..', 'html')
	server = subprocess.Popen(['python', '-m', 'SimpleHTTPServer'], cwd=path)

	rospy.init_node('ride')
	rospy.Service('/ride/node/create', ride.srv.NodeCreate, node_create)
	rospy.Service('/ride/node/destroy', ride.srv.NodeDestroy, node_destroy)
	rospy.Service('/ride/node/start', ride.srv.NodeStart, node_start)
	rospy.Service('/ride/node/stop', ride.srv.NodeStop, node_stop)
	rospy.Service('/ride/node/list', ride.srv.NodeList, node_list)
	rospy.Service('/ride/package/list', ride.srv.PackageList, cached_package_list)
	rospy.spin()

if __name__ == '__main__':
	main()
