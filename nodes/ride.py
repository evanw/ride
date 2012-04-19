#!/usr/bin/env python
import roslib; roslib.load_manifest('ride')

import os
import re
import json
import fcntl
import rospy
import signal
import pickle
import ride.srv
import subprocess
from std_msgs.msg import String

STATUS_STARTING = 0
STATUS_STARTED = 1
STATUS_STOPPING = 2
STATUS_STOPPED = 3

TOPIC_NAMES_TO_IGNORE = [] # ['/rosout']
NODE_NAMES_TO_IGNORE = [] # ['/rosout', '/rosbridge', '/ride']

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

class Node:
    class Slot:
        def __init__(self, ride, topic, is_input, node_name):
            self.ride = ride
            self.topic = topic
            self.is_input = is_input
            self.node_name = node_name
            self.ride.updates.create_slot(self)

        def __del__(self):
            self.ride.updates.destroy_slot(self)

    def __init__(self, ride, name):
        self.ride = ride
        self.name = name
        self.inputs = {}
        self.outputs = {}
        self.ride.updates.create_node(self)

    def __del__(self):
        self.ride.updates.destroy_node(self)

    def input(self, topic):
        if topic not in self.inputs:
            self.inputs[topic] = Node.Slot(self.ride, topic, True, self.name)
        return self.inputs[topic]

    def output(self, topic):
        if topic not in self.outputs:
            self.outputs[topic] = Node.Slot(self.ride, topic, False, self.name)
        return self.outputs[topic]

class OwnedNode:
    class Slot:
        def __init__(self, ride, topic, is_input, node_name, original_topic):
            self.ride = ride
            self.topic = topic
            self.is_input = is_input
            self.node_name = node_name
            self.original_topic = original_topic
            self.ride.updates.create_slot(self)

        def __del__(self):
            self.ride.updates.destroy_slot(self)

    def __init__(self, ride, name, path):
        self.ride = ride
        self.name = name
        self.inputs = {}
        self.outputs = {}
        self.path = path
        self.stdout = ''
        self.status = STATUS_STOPPED
        self.return_code = 0
        self.process = None
        self.remappings = {}
        self.ride.updates.create_node(self)
        self.ride.names_to_avoid.add(name)

    def __del__(self):
        self.ride.names_to_stop_avoiding.add(self.name)
        if self.process:
            self.process.send_signal(signal.SIGINT)
        self.ride.updates.destroy_node(self)

    def input(self, topic):
        if topic not in self.inputs:
            self.inputs[topic] = OwnedNode.Slot(self.ride, topic, True, self.name, self.remappings.get(topic, None))
        return self.inputs[topic]

    def output(self, topic):
        if topic not in self.outputs:
            self.outputs[topic] = OwnedNode.Slot(self.ride, topic, False, self.name, self.remappings.get(topic, None))
        return self.outputs[topic]

    def start(self):
        # Don't keep old processes around
        if self.process:
            self.process.kill()

        # Start off with the topic names
        map = {}
        published, subscribed = self.ride.db.topics.get(self.path, (set(), set()))
        for topic in published | subscribed:
            map[topic] = self.ride.unique_topic_name()
        self.remappings = dict(zip(map.values(), map.keys()))

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
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.status = STATUS_STARTING
        self.stdout = '$ ' + self.path + '\n'

        # Make sure self.process.stdout.read() won't block
        f = self.process.stdout
        fcntl.fcntl(f, fcntl.F_SETFL, fcntl.fcntl(f, fcntl.F_GETFL) | os.O_NONBLOCK)

    def stop(self):
        if self.process:
            self.process.send_signal(signal.SIGINT)
            self.status = STATUS_STOPPING

    def poll(self):
        if self.process:
            self.process.poll()
            try:
                self.stdout += self.process.stdout.read()
            except:
                pass
            if self.process.returncode is not None:
                self.status = STATUS_STOPPED
                self.return_code = self.process.returncode
                self.process = None

class Link:
    def __init__(self, ride, from_topic, to_topic):
        self.ride = ride
        self.from_topic = from_topic
        self.to_topic = to_topic
        self.name = self.ride.unique_node_name('ride_link')
        self.ride.names_to_avoid.add(self.name)
        command = ['rosrun', 'topic_tools', 'mux', to_topic, from_topic]
        command += ['__name:=' + self.name.replace('/', '')]
        self.process = subprocess.Popen(command)
        self.ride.updates.create_link(self)

    def __del__(self):
        self.ride.names_to_stop_avoiding.add(self.name)
        self.process.send_signal(signal.SIGINT)
        self.ride.updates.destroy_link(self)

class Updates:
    def __init__(self):
        self.pub = rospy.Publisher('/ride/updates', String)

    def send(self, data, send=True):
        if send:
            self.pub.publish(json.dumps(data))
        return data

    def create_node(self, node, send=True):
        data = {
            'type': 'create_node',
            'name': node.name,
            'is_owned': False,
        }
        if isinstance(node, OwnedNode):
            data['is_owned'] = True
        return self.send(data, send)

    def destroy_node(self, node, send=True):
        return self.send({
            'type': 'destroy_node',
            'name': node.name,
        }, send)

    def create_slot(self, slot, send=True):
        data = {
            'type': 'create_slot',
            'topic': slot.topic,
            'is_input': slot.is_input,
            'node_name': slot.node_name,
        }
        if isinstance(slot, OwnedNode.Slot):
            data['original_topic'] = slot.original_topic
        return self.send(data, send)

    def destroy_slot(self, slot, send=True):
        return self.send({
            'type': 'destroy_slot',
            'topic': slot.topic,
            'is_input': slot.is_input,
            'node_name': slot.node_name,
        }, send)

    def create_link(self, link, send=True):
        return self.send({
            'type': 'create_link',
            'from_topic': link.from_topic,
            'to_topic': link.to_topic,
        }, send)

    def destroy_link(self, link, send=True):
        return self.send({
            'type': 'destroy_link',
            'from_topic': link.from_topic,
            'to_topic': link.to_topic,
        }, send)

class RIDE:
    def __init__(self):
        self.db = DB()
        self.nodes = {}
        self.owned_nodes = {}
        self.links = {}
        self.updates = Updates()
        self.temporary_topic_count = 0

        # We want to avoid generating normal nodes for nodes that are links or
        # owned nodes. The problem is that once we delete links and owned nodes,
        # the actual underlying ROS node may continue to exist, at least for a
        # little while, while it's still shutting down. To fix this, we don't
        # create any normal nodes whose names are in the names_to_avoid set.
        # We don't want this set to grow indefinitely but links and owned nodes
        # can't remove themselves from the set when they are deleted because
        # the ROS nodes will live on afterwards. Instead, a name is flagged for
        # removal by putting it in the names_to_stop_avoiding set, and then the
        # main polling loop will remove it when the ROS node disappears.
        self.names_to_avoid = set()
        self.names_to_stop_avoiding = set()

        # Start up services (all member functions)
        rospy.Service('/ride/load', ride.srv.Load, self.load_service)
        rospy.Service('/ride/node/create', ride.srv.NodeCreate, self.node_create_service)
        rospy.Service('/ride/node/destroy', ride.srv.NodeDestroy, self.node_destroy_service)
        rospy.Service('/ride/node/start', ride.srv.NodeStart, self.node_start_service)
        rospy.Service('/ride/node/stop', ride.srv.NodeStop, self.node_stop_service)
        rospy.Service('/ride/link/create', ride.srv.LinkCreate, self.link_create_service)
        rospy.Service('/ride/link/destroy', ride.srv.LinkDestroy, self.link_destroy_service)

    def unique_topic_name(self):
        self.temporary_topic_count += 1
        return '/ride_temporary_topic_%d' % self.temporary_topic_count

    def unique_node_name(self, prefix):
        i = 2
        name = prefix = '/' + re.sub('[^A-Za-z0-9]', '_', prefix)
        names = set(self.nodes.keys() + self.owned_nodes.keys())
        while name in names:
            name = prefix + '_%d' % i
            i += 1
        return name

    def load_service(self, request):
        '''Implements the /ride/load service'''
        nodes = self.nodes.values() + self.owned_nodes.values()
        updates = []
        for node in nodes:
            updates.append(self.updates.create_node(node, False))
        for node in nodes:
            for slot in node.inputs.values() + node.outputs.values():
                updates.append(self.updates.create_slot(slot, False))
        for link in self.links.values():
            updates.append(self.updates.create_link(link))
        return ride.srv.LoadResponse(json.dumps({
            'updates': updates,
            'packages': self.db.packages,
        }))

    def node_create_service(self, request):
        '''Implements the /ride/node/create service'''
        name = self.unique_node_name(request.binary)
        path = self.db.path_of_binary(request.package, request.binary)
        self.owned_nodes[name] = OwnedNode(self, name, path)
        return ride.srv.NodeCreateResponse(name)

    def node_destroy_service(self, request):
        '''Implements the /ride/node/destroy service'''
        if request.name in self.owned_nodes:
            del self.owned_nodes[request.name]
            return ride.srv.NodeDestroyResponse(True)
        return ride.srv.NodeDestroyResponse(False)

    def node_start_service(self, request):
        '''Implements the /ride/node/start service'''
        if request.name in self.owned_nodes:
            self.owned_nodes[request.name].start()
            return ride.srv.NodeStartResponse(True)
        return ride.srv.NodeStartResponse(False)

    def node_stop_service(self, request):
        '''Implements the /ride/node/stop service'''
        if request.name in self.owned_nodes:
            self.owned_nodes[request.name].stop()
            return ride.srv.NodeStopResponse(True)
        return ride.srv.NodeStopResponse(False)

    def link_create_service(self, request):
        '''Implements the /ride/link/create service'''
        key = request.from_topic, request.to_topic
        if key not in self.links:
            self.links[key] = Link(self, request.from_topic, request.to_topic)
            return ride.srv.LinkCreateResponse(True)
        return ride.srv.LinkCreateResponse(False)

    def link_destroy_service(self, request):
        '''Implements the /ride/link/destroy service'''
        key = request.from_topic, request.to_topic
        if key in self.links:
            del self.links[key]
            return ride.srv.LinkDestroyResponse(True)
        return ride.srv.LinkDestroyResponse(False)

    def node(self, name):
        # Find the named node or create one if it doesn't exist yet
        if name in self.owned_nodes:
            return self.owned_nodes[name]
        if name in self.nodes:
            return self.nodes[name]
        if name not in self.names_to_avoid:
            self.nodes[name] = Node(self, name)
            return self.nodes[name]
        return None

    def poll(self):
        # Read the current system state from the ROS master
        publishers, subscribers, _ = map(dict, rospy.get_master().getSystemState()[2])
        node_names = set()

        # Link up published topics
        for topic in publishers:
            if topic in TOPIC_NAMES_TO_IGNORE:
                continue
            for name in publishers[topic]:
                if name in NODE_NAMES_TO_IGNORE:
                    continue
                node = self.node(name)
                if node is not None:
                    node.output(topic)
                node_names.add(name)

        # Link up subscribed topics
        for topic in subscribers:
            if topic in TOPIC_NAMES_TO_IGNORE:
                continue
            for name in subscribers[topic]:
                if name in NODE_NAMES_TO_IGNORE:
                    continue
                node = self.node(name)
                if node is not None:
                    node.input(topic)
                node_names.add(name)

        # Remove old node names
        killed_names = self.names_to_stop_avoiding - node_names
        self.names_to_stop_avoiding -= killed_names
        self.names_to_avoid -= killed_names

        # Remove old slots
        for name, node in self.nodes.items() + self.owned_nodes.items():
            for input in node.inputs.values():
                if input.topic not in subscribers or node.name not in subscribers[input.topic]:
                    del node.inputs[input.topic]
            for output in node.outputs.values():
                if output.topic not in publishers or node.name not in publishers[output.topic]:
                    del node.outputs[output.topic]

        # Remove old nodes
        for name in self.nodes.keys():
            if name not in node_names:
                del self.nodes[name]

        # Update owned nodes
        for name in self.owned_nodes:
            node = self.owned_nodes[name]

            # Check on the process
            if node.status == STATUS_STARTING and node.name in node_names:
                node.status = STATUS_STARTED
            node.poll()

            # Remember all observed topics for when we restart this node in the future
            if node.path not in self.db.topics:
                self.db.topics[node.path] = set(), set()
            published, subscribed = self.db.topics[node.path]
            published |= set(topic for topic in node.outputs if node.outputs[topic].original_topic is None)
            subscribed |= set(topic for topic in node.inputs if node.inputs[topic].original_topic is None)

        # Save all observed topics
        self.db.save()

class DB:
    def __init__(self):
        # Try to load information from the cache
        self.packages_path = os.path.join(os.path.dirname(__file__), '.package_cache')
        self.topics_path = os.path.join(os.path.dirname(__file__), '.topic_cache')
        print 'loading all packages...'
        self.packages = cache_load(self.packages_path, [])
        self.topics = cache_load(self.topics_path, {})
        if not self.packages:
            self.packages = self.list_packages()
            cache_save(self.packages_path, self.packages)
        print 'finished loading'

    def save(self):
        cache_save(self.topics_path, self.topics)

    def path_of_binary(self, package, binary):
        # Find the path of the binary (more secure than letting the
        # client do it, especially if the package list is limited)
        for info in self.packages:
            if info['name'] == package:
                for path in info['binaries']:
                    if path.endswith(binary):
                        return path

    def list_packages(self):
        # Find the names and locations of all packages
        lines = split_words(run('rospack', 'list'))
        packages = [{ 'name': name, 'path': path, 'binaries': [] } for name, path in lines]

        # Search for binaries using find (this is how rosrun does autocomplete)
        for i, package in enumerate(packages):
            print '[%d/%d]' % (i + 1, len(packages)),
            binaries = run('find', package['path'], '-type', 'f', '-perm', '+111')
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
                package['binaries'] = [line for line in lines if
                    all(not line.endswith(ext) for ext in extensions) and
                    '/CMakeFiles/' not in line and
                    'Makefile' not in line and
                    '/build/' not in line and
                    '/.svn/' not in line and
                    '/.git/' not in line]

        return packages

def main():
    # Start the node
    rospy.init_node('ride')
    ride = RIDE()

    # Start up a web server because there's no way to do that from roslaunch.
    # Unfortunately the SimpleHTTPServer API is so simple you can't set the
    # directory other than using the current directory, which we don't want
    # to mess with.
    path = os.path.join(os.path.dirname(__file__), '..', 'html')
    server = subprocess.Popen(['python', '-m', 'SimpleHTTPServer'], cwd=path)

    # Poll for changes to the ROS network
    while not rospy.is_shutdown():
        ride.poll()
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()
