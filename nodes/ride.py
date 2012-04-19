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
from ros import rosnode
from std_msgs.msg import String

STATUS_STARTING = 0
STATUS_STARTED = 1
STATUS_STOPPING = 2
STATUS_STOPPED = 3
STATUS_ERROR = 4

TOPIC_NAMES_TO_IGNORE = ['/rosout']
NODE_NAMES_TO_IGNORE = ['/rosout', '/rosbridge', '/ride']

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

    def __init__(self, ride, name, path, display_name):
        self.ride = ride
        self.name = name
        self.inputs = {}
        self.outputs = {}
        self.path = path
        self.display_name = display_name
        self.stdout = ''
        self.status = STATUS_STARTING
        self.return_code = None
        self.process = None
        self.remappings = {}
        self.ride.updates.create_node(self)
        self.ride.names_to_avoid.add(name)
        self.start()

    def __del__(self):
        self.ride.names_to_stop_avoiding.add(self.name)
        if self.process:
            self.ride.soft_kill_process(self.process)
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
            self.ride.soft_kill_process(self.process)

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
        self.stdout = '$ ' + self.path + '\n'
        try:
            # Start the node as a child process
            self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.status = STATUS_STARTING

            # Make sure self.process.stdout.read() won't block
            f = self.process.stdout
            fcntl.fcntl(f, fcntl.F_SETFL, fcntl.fcntl(f, fcntl.F_GETFL) | os.O_NONBLOCK)
        except Exception as e:
            # This will fail if the file doesn't exist (need to use rosmake again)
            self.stdout += str(e)
            self.status = STATUS_ERROR

        # Send the new status
        self.ride.updates.update_owned_node(self)

    def stop(self):
        if self.process:
            # Don't use self.ride.soft_kill_process(self.process) because we
            # are still checking for the return code in self.poll()
            if self.status == STATUS_STOPPING:
                self.process.kill()
            else:
                self.process.send_signal(signal.SIGINT)
                self.status = STATUS_STOPPING
                self.ride.updates.update_owned_node(self)

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
                self.ride.updates.update_owned_node(self)

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
        self.ride.updates.destroy_link(self)
        self.ride.soft_kill_process(self.process)

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
            data['display_name'] = node.display_name
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

    def update_owned_node(self, node, send=True):
        return self.send({
            'type': 'update_owned_node',
            'name': node.name,
            'status': node.status,
            'return_code': node.return_code,
        }, send)

class RIDE:
    def __init__(self):
        self.db = DB()
        self.nodes = {}
        self.owned_nodes = {}
        self.links = {}
        self.updates = Updates()
        self.unique_name_count = 0
        self.killed_processes = set()

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
        rospy.Service('/ride/node/output', ride.srv.NodeOutput, self.node_output_service)
        rospy.Service('/ride/link/create', ride.srv.LinkCreate, self.link_create_service)
        rospy.Service('/ride/link/destroy', ride.srv.LinkDestroy, self.link_destroy_service)

    def unique_topic_name(self):
        # This needs to be global because things don't work sometimes otherwise.
        # For example, if the generated names are in the "/ride_topics/"
        # namespace then ImageTransport puts "/camera_info" in
        # "/ride_topics/camera_info" and the remapping fails.
        self.unique_name_count += 1
        return '/ride_unique_topic_%d' % self.unique_name_count

    def unique_node_name(self, prefix):
        # BUGFIX: We can't reuse node names, so make them all different. Assume
        # no node will use any node name prefixed with "ride". If we don't prepend
        # "ride" then we run the risk of colliding with another node name. We could
        # test against all current node names, except there's a bug with creating
        # the same node twice.
        #
        # This function used to return the prefix plus the lowest number that made
        # the name unique, but you created and destroyed the same thing twice the
        # nodes had the same name. This should be fine since there's only one node
        # with that name active at a time, except that second node then becomes
        # immortal. Even after that node crashes, the topics it used are still
        # registered to its name. To unregister them I was using:
        #
        #     rosnode.cleanup_master_blacklist(roslib.scriptutil.get_master(), [name])
        #
        # but that doesn't throw an error and doesn't unregister the node. This
        # is odd because it looks like exactly what "rosnode cleanup" does. Running
        # "rosnode cleanup" from the terminal works, yet even if I run
        # rosnode.rosnode_cleanup() from ride.py it doesn't work. WTF.
        self.unique_name_count += 1
        prefix = re.sub('[^A-Za-z0-9]', '_', prefix)
        return '/ride_%s_%d' % (prefix, self.unique_name_count)

    def load_service(self, request):
        '''Implements the /ride/load service'''
        nodes = self.nodes.values() + self.owned_nodes.values()
        updates = []
        for node in nodes:
            updates.append(self.updates.create_node(node, False))
        for node in self.owned_nodes.values():
            updates.append(self.updates.update_owned_node(node, False))
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
        self.owned_nodes[name] = OwnedNode(self, name, path, request.binary + ' (' + request.package + ')')
        return ride.srv.NodeCreateResponse(True)

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

    def node_output_service(self, request):
        '''Implements the /ride/node/output service'''
        if request.name in self.owned_nodes:
            return ride.srv.NodeOutputResponse(self.owned_nodes[request.name].stdout, True)
        return ride.srv.NodeOutputResponse('', False)

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
        if name not in self.names_to_avoid and name not in NODE_NAMES_TO_IGNORE:
            self.nodes[name] = Node(self, name)
            return self.nodes[name]
        return None

    def soft_kill_process(self, process):
        # Kill a process we are about to forget about. Since it will be defunct
        # if we don't read it's exit status, we actually have to remember it and
        # keep polling it until it dies because that's how Unix processes work.
        process.send_signal(signal.SIGINT)
        self.killed_processes.add(process)

    def poll(self):
        # Read the current system state from the ROS master
        publishers, subscribers, _ = map(dict, rospy.get_master().getSystemState()[2])
        node_names = set(rosnode.get_node_names())

        # Create new nodes (needed for nodes without any pub/sub topics)
        for name in node_names:
            self.node(name)

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
                self.updates.update_owned_node(node)
            node.poll()

            # Remember all observed topics for when we restart this node in the future
            if node.path not in self.db.topics:
                self.db.topics[node.path] = set(), set()
            published, subscribed = self.db.topics[node.path]
            published |= set(topic for topic in node.outputs if node.outputs[topic].original_topic is None)
            subscribed |= set(topic for topic in node.inputs if node.inputs[topic].original_topic is None)

        # Save all observed topics
        self.db.save()

        # Keep polling killed child processes until they actually die
        for process in list(self.killed_processes):
            process.poll()
            if process.returncode is not None:
                self.killed_processes.remove(process)

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
