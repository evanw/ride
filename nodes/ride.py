#!/usr/bin/env python
import roslib; roslib.load_manifest('ride')

import os
import rospy
import ride.msg
import ride.srv
import subprocess
from ros import rosgraph
import rosgraph.masterapi

def run(*args):
	'''run the provided command and return its stdout'''
	print ' '.join(args)
	return subprocess.check_output(args).strip()

def split_words(text):
	'''return a list of lines where each line is a list of words'''
	return [line.strip().split() for line in text.split('\n')]

def node_start(request):
	pass

def node_stop(request):
	pass

def node_list(request):
	'''implements the /ride/node/list service'''
	nodes = {}
	publishers, subscribers, services = rosgraph.masterapi.Master('/ride').getSystemState()

	# Link up published topics
	for topic, names in publishers:
		for name in names:
			if name not in nodes:
				nodes[name] = ride.msg.Node(name, [], [], [])
			nodes[name].published.append(topic)

	# Link up subscribed topics
	for topic, names in subscribers:
		for name in names:
			if name not in nodes:
				nodes[name] = ride.msg.Node(name, [], [], [])
			nodes[name].subscribed.append(topic)

	# Link up provided services
	for service, names in services:
		for name in names:
			if name not in nodes:
				nodes[name] = ride.msg.Node(name, [], [], [])
			nodes[name].services.append(service)

	return ride.srv.NodeListResponse(nodes.values())

def package_list(request):
	'''implements the /ride/package/list service'''
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

	return ride.srv.PackageListResponse(packages)

def main():
	# Cache the package list because it's expensive to compute
	print 'loading all packages...'
	packages = package_list(None)
	print 'finished loading'

	# Start up a web server because there's no way to do that from roslaunch.
	# Unfortunately the SimpleHTTPServer API is so simple you can't set the
	# directory other than using the current directory, which we don't want
	# to mess with.
	path = os.path.join(os.path.dirname(__file__), '..', 'html')
	server = subprocess.Popen(['python', '-m', 'SimpleHTTPServer'], cwd=path)

	rospy.init_node('ride')
	rospy.Service('/ride/node/start', ride.srv.NodeStart, node_start)
	rospy.Service('/ride/node/stop', ride.srv.NodeStop, node_stop)
	rospy.Service('/ride/node/list', ride.srv.NodeList, node_list)
	rospy.Service('/ride/package/list', ride.srv.PackageList, lambda req: packages)
	rospy.spin()

if __name__ == '__main__':
	main()
