#!/usr/bin/env python

import os
import fnmatch
import re

from yaml import dump as yaml_dump
try:
	from yaml import CDumper as Dumper
except ImportError:
	from yaml import Dumper

class EmptyString(object):
	def __getattr__(self, name):
		return ''

try:
	from colorama import init as colorama_init
	from colorama import Fore, Style
except ImportError:
	def colorama_init(autoreset=False):
		pass

	Fore = EmptyString()
	Style = EmptyString()
	Back = EmptyString()

verbose = True

# Lists of the file extensions to check for rospy or roscpp nodes.
python_ext = ['py']
cpp_ext = ['c', 'cpp', 'cc', 'cxx']

#def write_yaml
def write_yaml(pynodes, cppnodes):
	output = {'nodes': \
				[{n['name']: {'type':'rosnode-py', 'path':n['path']}} for n in pynodes] \
				+ \
				[{n['name']: {'type':'rosnode-cpp', 'path':n['path']}} for n in cppnodes] \
			 }
	
	return yaml_dump(output, default_flow_style = False)

#def check_for_errors
def check_for_errors(pynodes, cppnodes):
	nodes = pynodes + cppnodes
	
	### Check for multiple nodes with the same name.
	duplicates = {}
	for node in nodes:
		if node['name'] in duplicates:
			duplicates[node['name']] += 1
		else:
			duplicates[node['name']] = 1
	
	for node, dups in duplicates.items():
		if dups > 1:
			print(Fore.YELLOW + "Warning: " + str(dups) + " nodes found with name '" + node + "'!")
	
	### Check for multiple nodes defined in the same file.
	duplicates = {}
	for node in nodes:
		if node['path'] in duplicates:
			duplicates[node['path']] += 1
		else:
			duplicates[node['path']] = 1
	
	for node, dups in duplicates.items():
		if dups > 1:
			print(Fore.YELLOW + "Warning: " + str(dups) + " nodes initialized in source file '" + node + "'!")
	
	return False

#def parse_cpp
def parse_cpp(files):
	cpp_rosincludes = []
	cpp_nodes = []
	
	# Loop through the input files.
	for f in files:
		included = False
		
		# Open the file for reading.
		fread = open(f, 'r')
		if fread != None:
			# If there are no errors, loop through the lines in the file.
			for line in fread:
				match = False
				if included:
					# If we find a ros::init call, mark down that there's a node in this
					# file.  Keep searching, because it's possible that multiple nodes are
					# defined in a single file.
					match = re.search('ros::init\(.+?, ?.+?, ?"(.+?)"\)', line)
					if match:
						cpp_nodes.append({'path':f, 'name':match.group(1)})
						continue
				# If we haven't included ros.h, look for the include call.
				else:
					match = re.search('#include ((<)?(")?)ros/ros.h(?(2)>)(?(3)")', line)
					if match:
						included = True
		# If included, add to cpp_rosincludes
		if included:
			cpp_rosincludes.append(f)
	
	if verbose:
		if len(cpp_rosincludes) > 0:
			print(Fore.BLUE + 'Found ' + str(len(cpp_rosincludes)) + ' c++ files that include ros.h:')
			print(Fore.BLUE + '    ' + ', '.join(cpp_rosincludes))
		else:
			print(Fore.BLUE + 'Found 0 c++ files that include ros.h.')
	
	# We've searched all files, so let's return our results.
	return cpp_nodes
					

### This function parses the python files to find rospy imports
### and calls to rospy.init_node.
def parse_py(files):
	py_rosimports = []
	py_nodes = []

	# Loop through the input files.
	for f in files:
		imported = False
		modname = 'rospy'
		
		# Open the file for reading.
		fread = open(f, 'r')
		if fread != None:
			# If there are no errors, loop through the lines in the file.
			for line in fread:
				match = False
				# If we've imported rospy, try to find an init_node call.
				if imported:
					match = re.search(modname + '.init_node\(\'(.+?)\'\)', line)
					# If we find a init_node call, mark down that there's a node in this
					# file.  Keep searching, because it's possible that multiple nodes are
					# defined in a single file.
					if match:
						py_nodes.append({'path':f, 'name':match.group(1)})
						continue
				# If we haven't imported rospy, look for the import.
				else:
					# Try first to match the import ... as ... statement.
					match = re.search('import rospy as (.+)', line)
					# If successful, save the name rospy was imported as.
					if match:
						imported = True
						modname = match.group(1)
					# If not, try to match 'import rospy'.
					else:
						match = re.search('import rospy', line)
						if match:
							imported = True
			
			# We've reached EOF.  Keep track of nodes that have been imported.
			if imported:
				py_rosimports.append(f)
	
	if verbose:
		print(Fore.BLUE + 'Found ' + str(len(py_rosimports)) + ' python files with rospy imports:')
		print(Fore.BLUE + '    ' + ', '.join(py_rosimports))
	
	# We've searched all files, so let's return our results.
	return py_nodes

### This function generates two lists, one of c++ source files
### recursively below this directory, and the second of python
### source files.  This is a "dumb" function - it only filters
### using file extensions, it doesn't infer type from content.
def gen_file_lists():
	file_lists = ([], [])
	for root, dirs, files in os.walk('.'):
		for pext in python_ext:
			for f in fnmatch.filter(files, '*.' + pext):
				file_lists[0].append(os.path.join(root, f))
		for cext in cpp_ext:
			for f in fnmatch.filter(files, '*.' + cext):
				file_lists[1].append(os.path.join(root, f))
	
	return file_lists

def main():
	# Initialize colorama for colored output.
	colorama_init(autoreset=True)
	
	# Get the list of files for consideration.
	py_files, cpp_files = gen_file_lists()
	
	# Filter the python files to the ones that call rospy.init_node().
	py_nodes = parse_py(py_files)
	# Filter the c++ files to the ones that call ros::init().
	cpp_nodes = parse_cpp(cpp_files)
	
	# If there are no errors, convert the node lists to YAML and write them to project.yaml.
	if not check_for_errors(py_nodes, cpp_nodes):
		projfile = open('project.yaml', 'w')
		projfile.write(write_yaml(py_nodes, cpp_nodes))
		
		print(Fore.GREEN + "RIDE node file generated without errors!")
		print(Style.BRIGHT + "Make sure to look at the generated file, as it is likely incorrect/incomplete.")

if __name__ == '__main__':
	main()
