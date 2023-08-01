#!/usr/bin/env python

"""
// Copyright (C) 2021 Lumotive
// Copyright 2023 HOKUYO AUTOMATIC CO.,LTD.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
"""

"""
	This is an helper script which generates the launch file / URDF file / RVIz file / config files when using Lumotive's M20 system

	List of generated files:
	- config / gen_multi_m20 / *
	- launch / gen_multi_m20_node_launcher.launch
	- urdf / gen_multi_m20.urdf
	- rviz / multi_m20 / gen_multi_m20.rviz

	To launch the driver after running this script, simply use roslaunch on gen_multi_m20_node_launcher.launch
"""

import argparse
import os
import sys
import shutil
import math
import copy
import subprocess
from io import open
import rospkg

if sys.version_info[0] < 3:
	from pathlib2 import Path
else:
	from pathlib import Path

try:
	from lxml import etree
except ModuleNotFoundError:
	print('[Lumotive FoV configs generator] Call failed. Please follow the instructions in the M20 user manual on how to add the lxml dependency using rosdep or run view_M20_all.sh.')
	exit()
try:
	import ruamel.yaml
except ModuleNotFoundError:
	print('[Lumotive FoV configs generator] Call failed. Please follow the instructions in the M20 user manual on how to add the ruamel.yaml dependency using rosdep or run view_M20_all.sh.')
	exit()

from ruamel.yaml.comments import CommentedMap as ordereddict  # to avoid '!!omap' in yaml


def parse_args():

	parser = argparse.ArgumentParser(description='Helper to generate launch/urdf/config/rviz files when using multiple M20 heads and FoVs.')

	parser.add_argument("input_arg",
						type=str,
						help="Specify the path to the YAML config file.")

	parser.add_argument("-s", "--split_fov_distance",
						required=False,
						default=0,
						type=float,
						help=argparse.SUPPRESS)

	return parser.parse_args()


class LaunchFileCreator:
	"""
		Class to generate XML launch file

		config_dict must a dict in which each key is a sensor port (a, b, c or d) and each value is a list of the desired FoVs (0 to 7)
	"""
	def __init__(self, config_dict, base_port, pkg_path=None):

		# This is how ports are attributed
		self.ports = {'a': {0: base_port, 1: base_port+1, 2: base_port+2, 3: base_port+3, 4: base_port+4, 5: base_port+5, 6: base_port+6, 7: base_port+7},
				 'b': {0: base_port+8, 1: base_port+9, 2: base_port+10, 3: base_port+11, 4: base_port+12, 5: base_port+13, 6: base_port+14, 7: base_port+15},
				 'c': {0: base_port+16, 1: base_port+17, 2: base_port+18, 3: base_port+19, 4: base_port+20, 5: base_port+21, 6: base_port+22, 7: base_port+23},
				 'd': {0: base_port+24, 1: base_port+25, 2: base_port+26, 3: base_port+27, 4: base_port+28, 5: base_port+29, 6: base_port+30, 7: base_port+31}}

		self.configs = config_dict

		# XML structures base
		self.launch_root = etree.Element("launch")

	def generate_XML(self, write_urdf=True, write_rviz=True):
		self.add_launch_header()
		self.add_launch_driver_instances()
		if write_urdf:
			self.add_URDF_entries()
		if write_rviz:
			self.add_rviz_entries()

	def add_launch_header(self):
		elem = etree.Element("arg")
		elem.set("name", "sensor_ip_")
		elem.set("default", "")
		elem.set("doc", "Lumotive sensor IP address. (Optional usage if pcap specified) It can be used to filter traffic when playing back a PCAP file.")
		
		self.launch_root.append(elem)
	
	def add_URDF_entries(self):
		self.launch_root.append(etree.Comment('URDF'))
		elem = etree.Element("arg")
		elem.set("name", "model")

		param_1 = etree.Element("param")
		param_1.set("name", "robot_description")
		param_1.set("textfile", "$(find lumotive_ros)/urdf/multi_m20.urdf")

		#param_2 = etree.Element("param")
		#param_2.set("name", "use_gui")
		#param_2.set("value", "true")

		self.launch_root.append(elem)
		self.launch_root.append(param_1)
		#self.launch_root.append(param_2)

	def add_rviz_entries(self):
		self.launch_root.append(etree.Comment('RVIZ'))

		joint_pub = etree.Element("node")
		joint_pub.set("name", "joint_state_publisher")
		joint_pub.set("pkg", "joint_state_publisher")
		joint_pub.set("type", "joint_state_publisher")

		elem = etree.Element("node")
		elem.set("name", "robot_state_publisher")
		elem.set("pkg", "robot_state_publisher")
		elem.set("type", "robot_state_publisher")
		
		self.launch_root.append(joint_pub)
		self.launch_root.append(elem)
		
		# reverse so 0 and 1 are on top
		for fov in reversed(range(4)):
			telem = etree.Element("node")
			telem.set("name", "rviz_" + str(fov))
			telem.set("pkg", "rviz")
			telem.set("type", "rviz")
			telem.set("args", "-d $(find lumotive_ros)/rviz/multi_m20/multi_m20_" + str(fov) + ".rviz")
			telem.set("required", "true")
			self.launch_root.append(telem)

	def add_launch_driver_instances(self):
		# Create lumotive ROS driver entries using configs
		for sensor_head in self.configs:
			for fov_num in self.configs[sensor_head]:
				# remaps
				remap_0 = etree.Element("remap")
				remap_0.set("from", "/lumotive_ros/pointcloud")
				remap_0.set("to", "/lumotive_ros/pointcloud_"+sensor_head+"_"+str(fov_num))

				# lumotive_driver instance
				driver_node = etree.Element("node")
				driver_node.set("pkg", "lumotive_ros")
				driver_node.set("name", "lumotive_driver_"+sensor_head+"_"+str(fov_num))
				driver_node.set("type", "lumotive_driver")
				driver_node.set("output", "screen")

				param_0 = etree.Element("rosparam")
				param_0.set("command", "load")
				param_0.set("file", "$(find lumotive_ros)/config/gen_multi_m20/m20_configs_"+sensor_head+"_"+str(fov_num)+".yaml")

				param_1 = etree.Element("param")
				param_1.set("name", "sensor_ip")
				param_1.set("type", "str")
				param_1.set("value", "$(arg sensor_ip_)")

				param_2 = etree.Element("param")
				param_2.set("name", "sensor_port")
				param_2.set("type", "int")
				param_2.set("value", str(self.ports[sensor_head][fov_num]))

				driver_node.append(param_0)
				driver_node.append(param_1)
				driver_node.append(param_2)

				self.launch_root.append(etree.Comment('Instances for Sensor Head ' + sensor_head + ' - FoV ' + str(fov_num)))
				self.launch_root.append(remap_0)
				self.launch_root.append(driver_node)

	def print_to_console(self):
		print(etree.tostring(self.launch_root, pretty_print=True))

	def write_to_file(self, filename):
		et = etree.ElementTree(self.launch_root)
		et.write(filename, pretty_print=True)


class URDFFileCreator:
	"""
		Class to generate XML URDF file

		config_dict must a dict in which each key is a sensor port (a, b, c or d) and each value is a list of the desired FoVs (0 to 7)
	"""
	def __init__(self, config_dict, positions, visual_positions, fov_split_distance=0.0, robot_name="4_m20s", pkg_path=None):
		self.configs = config_dict
		self.positions = positions
		self.visual_origins = visual_positions
		self.fov_split_distance = fov_split_distance

		# XML structures base
		self.robot_root = etree.Element("robot")
		self.robot_root.set("name", robot_name)

	def generate_XML(self):
		self.generate_links()
		self.generate_joints()

	def generate_links(self):
		colors_name = {"a": "blue", "b": "red", "c": "green", "d": "yellow"}
		colors = {"a": "0 0 1 1", "b": "1 0 0 1", "c": "0 1 0 1", "d": "1 1 0 1"}
		link = etree.Element("link")
		link.set("name", "assembly_base")
		self.robot_root.append(link)

		for sensor_head in self.configs:
			for fov_num in self.configs[sensor_head]:
				link = etree.Element("link")
				link.set("name", "m20_"+sensor_head+"_"+str(fov_num))
				visual = etree.Element("visual")

				origin = etree.Element("origin")
				xyz = self.visual_origins[sensor_head+"_vis"][0:3]
				xyz_str = ' '.join(map(str, xyz))
				rpy = self.visual_origins[sensor_head+"_vis"][3:6]
				rpy_str = ' '.join(map(str, rpy))
				origin.set("xyz", xyz_str)
				origin.set("rpy", rpy_str)

				geometry = etree.Element("geometry")
				mesh = etree.Element("mesh")
				mesh.set("filename", "package://lumotive_ros/meshes/m20_lidar.STL")
				geometry.append(mesh)

				material = etree.Element("material")
				material.set("name", colors_name[sensor_head])
				color = etree.Element("color")
				color.set("rgba", colors[sensor_head])
				material.append(color)

				visual.append(origin)
				visual.append(geometry)
				visual.append(material)

				link.append(visual)

				self.robot_root.append(link)

	def generate_joints(self):
		current_yaw = 0.0
		for sensor_head in self.configs:
			for fov_num in self.configs[sensor_head]:
				joint = etree.Element("joint")
				joint.set("name", "m20_"+sensor_head+"_"+str(fov_num)+"_joint")
				joint.set("type", "fixed")
				parent = etree.Element("parent")
				parent.set("link", "assembly_base")
				child = etree.Element("child")
				child.set("link", "m20_"+sensor_head+"_"+str(fov_num))
				origin = etree.Element("origin")

				xyz = self.positions[sensor_head+"_pos"][0:3]
				xyz[2] += fov_num*self.fov_split_distance
				xyz_str = ' '.join(map(str, xyz))
				rpy = self.positions[sensor_head+"_pos"][3:6]
				rpy_str = ' '.join(map(str, rpy))
				origin.set("xyz", xyz_str)
				origin.set("rpy", rpy_str)
				joint.append(parent)
				joint.append(child)
				joint.append(origin)
				self.robot_root.append(joint)

			current_yaw += math.pi / 2

	def write_to_file(self, filename):
		et = etree.ElementTree(self.robot_root)
		et.write(filename, pretty_print=True)


class ConfigFilesCreator:
	"""
		Class to generate the config files

		config_dict must a dict in which each key is a sensor port (a, b, c or d) and each value is a list of the desired FoVs (0 to 7)
	"""
	def __init__(self, config_dict, pkg_path=None):
		self.yaml = ruamel.yaml.YAML()
		self.yaml.preserve_quotes = True
		self.configs = config_dict
		self.pkg_path = pkg_path
		self.configs_base_content = self.yaml.load(open(os.path.join(self.pkg_path, 'config', 'm20_configs.yaml'), 'r')) # Use base file

	def dump_config_files(self, directory):

		if not os.path.exists(directory):
			os.makedirs(directory)
		else:
			shutil.rmtree(directory)
			os.makedirs(directory)

		for sensor_head in self.configs:
			for fov_num in self.configs[sensor_head]:
				current_config = self.configs_base_content
				current_config['device_frame_id'] = "m20_"+sensor_head+"_"+str(fov_num)
				self.yaml.dump(current_config, open(os.path.join(directory, 'm20_configs_'+sensor_head+'_'+str(fov_num)+'.yaml'), 'w+', encoding='utf8'))


class RVIzFileCreator:
	"""
		Class to generate the rviz config file

		config_dict must a dict in which each key is a sensor port (a, b, c or d) and each value is a list of the desired FoVs (0 to 7)
	"""
	def __init__(self, config_dict, pkg_path=None):
		self.yaml = ruamel.yaml.YAML()
		self.yaml.preserve_quotes = True
		self.configs = config_dict
		self.pkg_path = pkg_path
		self.rviz_base_content = self.yaml.load(open(os.path.join(self.pkg_path, 'rviz', 'multi_m20', 'multi_m20_base_config.rviz'), 'r'))
		self.link_base = ordereddict([('Alpha', 1), ('Show Axes', False), ('Show Trail', False), ('Value', True)])
		self.single_pointcloud2_display = ordereddict([('Alpha', 1), ('Autocompute Intensity Bounds', False), ('Autocompute Value Bounds', ordereddict([('Max Value', 10), ('Min Value', -10), ('Value', True)])), ('Axis', 'Z'), ('Channel Name', 'intensity'), ('Class', 'rviz/PointCloud2'), ('Color', '255; 255; 255'), ('Color Transformer', 'Intensity'), ('Decay Time', 0), ('Enabled', True), ('Invert Rainbow', False), ('Max Color', '255; 255; 255'), ('Max Intensity', 2000), ('Min Color', '0; 0; 0'), ('Min Intensity', 0), ('Name', 'PointCloud2'), ('Position Transformer', 'XYZ'), ('Queue Size', 10), ('Selectable', True), ('Size (Pixels)', 2), ('Size (m)', 0.014000000432133675), ('Style', 'Points'), ('Topic', '/lumotive_ros/pointcloud_c'), ('Unreliable', False), ('Use Fixed Frame', True), ('Use rainbow', True), ('Value', True)])

	def dump_rviz_config_file(self, directory):
		for sensor_head in self.configs:
			for fov_num in self.configs[sensor_head]:
				pt2_instance = self.single_pointcloud2_display
				pt2_instance['Name'] = "PointCloud_"+sensor_head+"_"+str(fov_num)
				pt2_instance['Topic'] = "/lumotive_ros/pointcloud_"+sensor_head+"_"+str(fov_num)
				self.rviz_base_content['Visualization Manager']['Displays'][1]['Links']["m20_"+sensor_head+"_"+str(fov_num)] = copy.deepcopy(self.link_base)
				self.rviz_base_content['Visualization Manager']['Displays'].append(copy.deepcopy(pt2_instance))

		self.yaml.dump(self.rviz_base_content, open(os.path.join(directory, 'gen_multi_m20.rviz'), 'w+', encoding='utf8'))


class RvizTiledFileCreator:
	"""Class to generate multiple tiled rviz config files

	config_dict must a dict in which each key is a sensor port (a, b, c or d)
	and each value is a list of the desired FoVs (0 to 7)
	"""
	def __init__(self, config_dict, pkg_path=None):
		self.yaml = ruamel.yaml.YAML()
		self.yaml.preserve_quotes = True
		self.configs = config_dict
		self.pkg_path = pkg_path
		self.rviz_base_content = self.yaml.load(open(os.path.join(self.pkg_path, 'rviz', 'multi_m20', 'multi_m20_base_config.rviz'), 'r'))
		self.link_base = ordereddict([('Alpha', 1), ('Show Axes', False), ('Show Trail', False), ('Value', True)])
		self.single_pointcloud2_display = ordereddict(
			[('Alpha', 1),
			 ('Autocompute Intensity Bounds', False),
			 ('Autocompute Value Bounds', ordereddict([('Max Value', 10),
													   ('Min Value', -10),
													   ('Value', True)])),
			 ('Axis', 'Z'),
			 ('Channel Name', 'intensity'),
			 ('Class', 'rviz/PointCloud2'),
			 ('Color', '255; 255; 255'),
			 ('Color Transformer', 'Intensity'),
			 ('Decay Time', 0),
			 ('Enabled', True),
			 ('Invert Rainbow', False),
			 ('Max Color', '255; 255; 255'),
			 ('Max Intensity', 2000),
			 ('Min Color', '0; 0; 0'),
			 ('Min Intensity', 0),
			 ('Name', 'PointCloud2'),
			 ('Position Transformer', 'XYZ'),
			 ('Queue Size', 10),
			 ('Selectable', True),
			 ('Size (Pixels)', 2),
			 ('Size (m)', 0.014000000432133675),
			 ('Style', 'Points'),
			 ('Topic', '/lumotive_ros/pointcloud_c'),
			 ('Unreliable', False),
			 ('Use Fixed Frame', True),
			 ('Use rainbow', True),
			 ('Value', True),
			])

	def dump_rviz_config_file(self, directory):
		for fov_num in range(4):
			starting_base_content = copy.deepcopy(self.rviz_base_content)
			pt2_instance = copy.deepcopy(self.single_pointcloud2_display)
			for sensor_head in self.configs:

				pt2_instance['Name'] = "PointCloud_"+sensor_head+"_"+str(fov_num)
				pt2_instance['Topic'] = "/lumotive_ros/pointcloud_"+sensor_head+"_"+str(fov_num)
				starting_base_content['Visualization Manager']['Displays'][1]['Links']["m20_"+sensor_head+"_"+str(fov_num)] = copy.deepcopy(self.link_base)
				starting_base_content['Visualization Manager']['Displays'].append(copy.deepcopy(pt2_instance))


			# Window size
			# width
			pw = subprocess.check_output("xrandr | awk -F'[ +]' '/primary/{print $4}' | cut -d x -f1",
										 shell=True)
			width = int(pw)
			w = int(width / 2.25)
			ph = subprocess.check_output("xrandr | awk -F'[ +]' '/primary/{print $4}' | cut -d x -f2",
										 shell=True)
			height = int(ph)
			h = int(height / 2.25)
			x = int((fov_num >> 1) * width // 2)
			y = int((fov_num & 0b1) * height // 2 + ((fov_num & 0b1) * height * .15))
			#print('fw fh w h x y', width, height, w, h, x, y)
			# Adjust window size and location based on which FOV num

			starting_base_content['Window Geometry']['Width'] = w
			starting_base_content['Window Geometry']['Height'] = h

			starting_base_content['Window Geometry']['X'] = x
			starting_base_content['Window Geometry']['Y'] = y
			starting_base_content['Window Geometry']['Hide Left Dock'] = True
			starting_base_content['Window Geometry']['Hide Right Dock'] = True
			starting_base_content['Window Geometry']['Displays']['collapsed'] = True
			starting_base_content['Window Geometry']['Views']['collapsed'] = True

			self.yaml.dump(starting_base_content, open(
				os.path.join(directory, 'multi_m20' + '_' + str(fov_num) + '.rviz'),
				'w+', encoding='utf8'))


if __name__ == "__main__":
	args_ = parse_args()

	input_arg = args_.input_arg
	split_fov_distance = args_.split_fov_distance

	base_port = 10940
	all_configuration = {'a': [0, 1, 2, 3, 4, 5, 6, 7],
						'b': [0, 1, 2, 3, 4, 5, 6, 7],
						'c': [0, 1, 2, 3, 4, 5, 6, 7],
						'd': [0, 1, 2, 3, 4, 5, 6, 7]}

	# Sensor head keys
	sensor_head_keys = list(all_configuration.keys())
	sensor_positions_keys = [s + '_pos' for s in sensor_head_keys]
	sensor_visual_keys = [s + '_vis' for s in sensor_head_keys]

	yaml = ruamel.yaml.YAML()
	full_configuration = dict(yaml.load(open(input_arg, 'r')))
	configuration = {key: full_configuration[key] for key in sensor_head_keys if key in full_configuration.keys()}
	positions = {key: full_configuration[key] for key in sensor_positions_keys if key in full_configuration.keys()}
	visual_positions = {key: full_configuration[key] for key in sensor_visual_keys if key in full_configuration.keys()}

	################################################################################################
	# Check the input data
	for sensor_head in configuration.keys():
		if sensor_head not in all_configuration.keys():
			raise Exception("Sensor head '" + sensor_head + "' is not valid. Valid sensor heads are: " + str(list(all_configuration.keys())))
		for fov_num in configuration[sensor_head]:
			if fov_num not in all_configuration[sensor_head]:
				raise Exception("Sensor head " + sensor_head + " has no FoV " + str(fov_num) + ". Valid FoVs for sensor head " + sensor_head + " are: " + str(all_configuration[sensor_head]))
		if sensor_head + "_pos" not in positions.keys():
			raise Exception("Sensor head " + sensor_head + " has no position specified (no " + sensor_head + "_pos key)")
		if sensor_head + "_vis" not in visual_positions.keys():
			raise Exception("Sensor head " + sensor_head + " has no visual origin specified (no " + sensor_head + "_vis key)")
	################################################################################################

	# Find lumotive_ros path
	rospack = rospkg.RosPack()
	pkg_top = rospack.get_path('lumotive_ros')

	# M20 sensor heads / FoVs to activate
	launch_creator = LaunchFileCreator(configuration, base_port, pkg_path=pkg_top)
	launch_creator.generate_XML()
	Path(os.path.join(pkg_top, 'launch')).mkdir(parents=False, exist_ok=True)
	launch_creator.write_to_file(os.path.join(pkg_top, 'launch', 'multi_m20_node_launcher.launch'))

	urdf_creator = URDFFileCreator(configuration, positions, visual_positions, fov_split_distance=split_fov_distance, pkg_path=pkg_top)
	urdf_creator.generate_XML()
	Path(os.path.join(pkg_top, 'urdf')).mkdir(parents=False, exist_ok=True)
	urdf_creator.write_to_file(os.path.join(pkg_top, 'urdf', 'multi_m20.urdf'))

	configs_creator = ConfigFilesCreator(configuration, pkg_path=pkg_top)
	Path(os.path.join(pkg_top, 'config', 'gen_multi_m20')).mkdir(parents=False, exist_ok=True)
	configs_creator.dump_config_files(os.path.join(pkg_top, 'config', 'gen_multi_m20'))

	# rviz_creator = RVIzFileCreator(configuration, pkg_path=pkg_top)
	rviz_creator = RvizTiledFileCreator(configuration, pkg_path=pkg_top)
	Path(os.path.join(pkg_top, 'rviz', 'multi_m20')).mkdir(parents=False, exist_ok=True)
	rviz_creator.dump_rviz_config_file(os.path.join(pkg_top, 'rviz', 'multi_m20'))

	print('[Lumotive FoV configs generator] FoV launch, config, urdf and rviz files were generated with success!')
