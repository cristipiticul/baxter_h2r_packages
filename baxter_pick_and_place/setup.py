#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['baxter_pick_and_place'],
    package_dir={'': 'src'},
    requires = ["rospy", 
			    "geometry_msgs", 
			    "std_msgs", 
			    "baxter_core_msgs", 
			    "dynamic_reconfigure", 
			    "baxter_examples", 
			    'moveit_commander', 
			    'moveit_msgs', 
			    'visualization_msgs', 
			    'tf']
)

setup(**d)