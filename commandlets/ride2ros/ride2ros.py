#!/usr/bin/env python

import os

from yaml import safe_load as yaml_load
try:
	from yaml import CLoader as Loader
except ImportError:
	from yaml import Loader

try:
	from colorama import init as colorama_init
	from colorama import Fore, Style
except ImportError:
	class EmptyString(object):
		def __getattr__(self, name):
			return ''
	
	def colorama_init(autoreset=False):
		pass

	Fore = EmptyString()
	Style = EmptyString()
	Back = EmptyString()

workspace_dir = os.path.expanduser('~/ros_ide/workspace')

def main():
	# Init output coloring library.
	colorama_init(autoreset=True)
	
	# Try to open the project file.
	try:
		proj_file = file('./project.yaml', 'r')
	except IOError:
		print(Fore.RED + 'Error: Couldn\'t find project.yaml file in current directory!');
		return
	
	# If we opened the file, parse the yaml.
	project = yaml_load(proj_file)
	pkg_name = os.path.basename(os.getcwd())
	
	output = []
	# Start the roslaunch file.
	# NOTE: The brackets around the string are VERY important.  Without them, it
	# 		will add the characters of the string to output one by one instead of
	#		adding the string as a whole.
	output += ['<launch>']
	
	# Define the core ros parameters.  We do this to add the workspace directory onto
	# ROS's package path.
	output += ['\t<machine name="local-ride" address="localhost" default="true" ' + \
				'ros-root="$(env ROS_ROOT)" ros-package-path="$(env ROS_PACKAGE_PATH):' + workspace_dir + '" />']
	
	# Loop through the nodes listed in the project file.
	for node in project['nodes']:
		# Create a string to build the node XML into.
		node_xml = '\t'
		shorthand = True
		
		# Add lines for launching nodes.
		if 'exec' in node:
			node_xml += '<node '
			node_xml += 'name="' + node['name'] + '" '
			if 'pkg' in node:
				node_xml += 'pkg="' + node['pkg'] + '" '
			else:
				node_xml += 'pkg="' + pkg_name + '" '
				node_xml += 'output="screen" '
			node_xml += 'type="' + node['exec'] + '" '
			if 'chdir' in node:
				if node['chdir'] == True:
					node_xml += 'cwd="node" '
		elif 'launch' in node:
			node_xml += '<include '
			node_xml += 'file="$(find ' + node['pkg'] + ')/' + node['launch'] + '" '
		else:
			print(Fore.YELLOW + 'Warning: node "' + node['name'] + \
					'" has no launch configuration, and therefore will not be run when package is launched.')
			continue
		
		# Figure out whether we're using shorthand or not.
		if 'remap' in node or 'params' in node:
			shorthand = False
			node_xml = node_xml[:-1] + '>\n'
		
		# Handle remapping.
		if 'remap' in node:
			for mapping in node['remap']:
				node_xml += '\t\t<remap from="' + mapping[0] + '" to="' + mapping[1] +'" />\n'
		
		if 'params' in node:
			for param in node['params']:
				node_xml += '\t\t<param name="' + param['name'] + '" value="' + str(param['value']) + '" />\n'
		
		# Close the tag.
		if shorthand:
			node_xml += '/>'
		else:
			if 'exec' in node:
				node_xml += '\t</node>'
			elif 'launch' in node:
				node_xml += '\t</include>'
		
		# Add the xml for this node to the output string.
		output += [node_xml]
	
	# End the roslaunch file.
	output += ['</launch>']
	
	output_path = pkg_name + '.launch'
	try:
		output_file = open(output_path, 'w')
		output_file.write('\n'.join(output))
		print(Fore.GREEN + 'roslaunch file successfully written to "' + output_path + '"!');
	except IOError:
		pass

if __name__ == '__main__':
	main()
