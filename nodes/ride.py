#!/usr/bin/env python
import roslib; roslib.load_manifest('ride')

import re
import os
import sys
import fcntl
import rospy
import pickle
import signal
import ride.msg
import ride.srv
import tempfile
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

def cache_load(path, default):
    '''make pickle.load() easier to use'''
    try:
        return pickle.load(open(path))
    except:
        return default

def cache_save(path, obj):
    '''make pickle.dump() easier to use'''
    try:
        pickle.dump(obj, open(path, 'w'))
    except:
        pass

# Map of node names to Node instances
owned_nodes = {}

# List of all active node names (for generating new unique names)
node_names = []

# List of ride.msg.Package instances
packages = []

# Map of binary_path to (set(published_topics), set(subscribed_topics))
topics = {}

# Other globals
topics_path = ''
temporary_topic_count = 0

# This seems to need to be global? If the generated names are in the /ride
# namespace then ImageTransport puts /camera_info in /ride/camera_info and
# the remapping fails. Leaving these in the global namespace for now.
temporary_topic_prefix = '/ride_temporary_topic'

class Node:
    def __init__(self, package, binary, name):
        self.package = package
        self.binary = binary
        self.name = name
        self.proc = None
        self.path = None
        self.return_code = None
        self.is_starting = True
        self.output = ''

        # Find the path of the binary (more secure than letting the
        # client do it, especially if the package list is limited)
        for p in packages:
            if p.name == package:
                for b in p.binaries:
                    if b.endswith(binary):
                        self.path = b

    def __del__(self):
        self.stop()

    def start(self):
        if not self.path:
            return

        # Kill the node if it exists
        if self.proc:
            self.proc.kill()
            self.proc = None

        #                       _   _    _    ____ _  __
        #                      | | | |  / \  / ___| |/ /
        #                      | |_| | / _ \| |   | ' /
        #                      |  _  |/ ___ \ |___| . \
        #                      |_| |_/_/   \_\____|_|\_\
        #
        # The goal of this IDE is to dynamically reconnect ROS nodes while the
        # graph is running. This requires remapping topic names on the fly.
        # While ROS supports remappings, they can only be done before publisher
        # objects are created (i.e. only at initialization time) and cannot be
        # updated at runtime. To do runtime editing of the graph, we need to
        # remap all possible topic names at the start and then dynamically add
        # and remove special forwarder nodes that implement the connections.
        #
        # The problem is that we don't know all possible topic names. Instead
        # we do the next best thing, which is to remember all topic names we
        # have ever seen during a node's lifetime. Then, the next time that
        # node is run, we will be able to remap those topics and do dynamic
        # connections.
        #
        # While in theory we can just remap all observed topics, this doesn't
        # always work because some nodes derive topic names using the name of
        # another topic as a prefix. For example, the ar_recog node publishes
        # to topics including /ar/image and /ar/image/theora. We can try to
        # remap these using /ar/image/theora:=/t1 and /ar/image:=t2 but since
        # the /theora is appended to /ar/image after it has been remapped,
        # the generated topic name will be /t2/theora and our remapping to t1
        # will fail. To account for this, we automatically add both remappings
        # in this case (/ar/image/theora:=/t1 and /t2/theora:=/t1).

        # Start off with the topic names
        map = {}
        published, subscribed = topics.get(self.path, (set(), set()))
        for topic in published | subscribed:
            map[topic] = temporary_topic_name()

        # Find prefixes and add those remappings too
        keys = map.keys()
        for prefix in keys:
            for topic in keys:
                if topic.startswith(prefix) and topic != prefix:
                    map[map[prefix] + topic[len(prefix):]] = map[topic]

        # Build up the command
        command = [self.path, '__name:=' + self.name.replace('/', '')]
        for topic in map:
            command.append(topic + ':=' + map[topic])

        # Start the node again
        self.proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.is_starting = True
        self.output = '$ ' + self.path + '\n'

        # Make sure self.proc.stdout.read() won't block
        f = self.proc.stdout
        fcntl.fcntl(f, fcntl.F_SETFL, fcntl.fcntl(f, fcntl.F_GETFL) | os.O_NONBLOCK)

    def stop(self):
        if self.proc:
            self.proc.send_signal(signal.SIGINT)

    def update(self):
        if self.proc:
            self.proc.poll()
            try:
                self.output += self.proc.stdout.read()
            except:
                pass
            if self.proc.returncode is not None:
                self.return_code = self.proc.returncode
                self.proc = None

def temporary_topic_name():
    global temporary_topic_count
    topic = temporary_topic_prefix + str(temporary_topic_count)
    temporary_topic_count += 1
    return topic

def unique_name(like):
    like = '/' + re.sub('[^A-Za-z0-9]', '_', like)
    names = set(node_names)
    if like not in names:
        return like
    i = 2
    while True:
        name = like + str(i)
        if name not in names:
            return name
        i += 1

def node_create(request):
    '''implements the /ride/node/create service'''
    name = unique_name(request.binary)
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

def node_output(request):
    '''implements the /ride/node/output service'''
    if request.name in owned_nodes:
        data = owned_nodes[request.name].output
        return ride.srv.NodeOutputResponse(request.id, data, True)
    return ride.srv.NodeOutputResponse(request.id, '', False)

def node_list(request):
    '''implements the /ride/node/list service'''

    # Query the master for the entire state of the system
    node_map = {}
    owned_node_map = {}
    publishers, subscribers, services = rosgraph.masterapi.Master('/ride').getSystemState()

    # Link up published topics
    for topic, names in publishers:
        for name in names:
            if name not in node_map:
                node_map[name] = ride.msg.Node(name, [], [], [])
            node_map[name].published.append(topic)

    # Link up subscribed topics
    for topic, names in subscribers:
        for name in names:
            if name not in node_map:
                node_map[name] = ride.msg.Node(name, [], [], [])
            node_map[name].subscribed.append(topic)

    # Link up provided services
    for service, names in services:
        for name in names:
            if name not in node_map:
                node_map[name] = ride.msg.Node(name, [], [], [])
            node_map[name].services.append(service)

    # Make sure all owned nodes are returned and are marked as owned
    for name in owned_nodes:
        # Check on the process
        o = owned_nodes[name]
        o.update()

        # Make sure we return an OwnedNode, even for running nodes
        n = ride.msg.OwnedNode(name, [], [], [], 0, 0)
        owned_node_map[name] = n
        n.return_code = 0 if o.return_code is None else o.return_code
        if name in node_map:
            # Move the state over from the other node
            n.published = node_map[name].published
            n.subscribed = node_map[name].subscribed
            n.services = node_map[name].services
            n.status = ride.msg.OwnedNode.STATUS_RUNNING
            o.is_starting = False
            del node_map[name]

            # Remember all observed topics for when we restart this node in the future
            if o.path not in topics:
                topics[o.path] = set(), set()
            p, s = topics[o.path]
            p |= set(t for t in n.published if not t.startswith(temporary_topic_prefix))
            s |= set(t for t in n.subscribed if not t.startswith(temporary_topic_prefix))
        elif not o.proc and o.return_code is not None:
            n.status = ride.msg.OwnedNode.STATUS_STOPPED
        elif o.is_starting:
            n.status = ride.msg.OwnedNode.STATUS_STARTING
        else:
            n.status = ride.msg.OwnedNode.STATUS_STOPPING

    # Save current node names for when we need to generate a new unique name
    global node_names
    node_names = node_map.keys() + owned_node_map.keys()

    # Save map of observed topics
    cache_save(topics_path, topics)

    return ride.srv.NodeListResponse(request.id, node_map.values(), owned_node_map.values())

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
    # Load cached information (packages and topics)
    global packages, topics_path, topics
    packages_path = os.path.join(os.path.dirname(__file__), '.package_cache')
    topics_path = os.path.join(os.path.dirname(__file__), '.topic_cache')
    print 'loading all packages...'
    packages = cache_load(packages_path, [])
    topics = cache_load(topics_path, {})
    if not packages:
        packages = package_list(ride.srv.PackageListRequest('')).packages
        cache_save(packages_path, packages)
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
    rospy.Service('/ride/node/output', ride.srv.NodeOutput, node_output)
    rospy.Service('/ride/package/list', ride.srv.PackageList, cached_package_list)
    rospy.spin()

if __name__ == '__main__':
    main()
