# Introduction

This node contains routines using high-level actions provided by o2as_skills and o2as_vision. It is meant to enable quick prototyping and to represent task instructions in a simple and compact way.

# Structure

- common_functions.py offers convenience functions that access other services and actions  
- taskboard.py, assembly.py, kitting.py should contain the full set of instructions for each task
- calibration_taskboard.py, calibration_assembly.py, calibration_kitting.py should be used to check the calibration for each scene (e.g. move the robots to the corners of each container)

# Notes

The Python script should be used mostly as a wrapper to the actions provided by o2as_skill_server. If anything detailed, (such as velocity adjustments during picking, or gripper opening parameters) need to be implemented, it should be done in o2as_skill_server, and not in this package.