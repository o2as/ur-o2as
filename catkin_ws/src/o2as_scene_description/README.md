# Introduction

This package contains the description of our robot system and scenes. The part definitions can be found in o2as_parts_description. 

It also contains URDF generators for the kitting scene bin arrangement and the placement mat of the taskboard scene.

# How to change the scene of a certain task (kitting, taskboard, assembly)

If you want to adjust the positions of parts that are specific to your task, change only the "kitting_scene.xacro", "assembly_scene.xacro" and "taskboard_scene.xacro" files.

The task will be set via a parameter that is to be announced.

# How to change the bin arrangement of the kitting task

You can change the bin layout for the kitting task easily by changing "placement_bin_layout.csv","set_origin.csv" and "placement_bin_definitions.csv". Then, execute: `rosrun o2as_scene_description bin_layout_generator.py` which updates "/home/osx/ur-o2as/catkin_ws/src/o2as_scene_description/urdf/kitting_scene_old.xacro"


# Known issues

Increasing the size of the boxes too much causes them to be unstable in the Gazebo simulation.
