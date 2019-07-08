# Introduction

This package read URDF files and converts them to CollisionObject messages which can be used in C++ code.

This is meant to simplify the definition of collision objects and named frames on them.

NOTE: The URDF file must consist of one root link with one visual geometry object, and child links that define frames of interest on the object. All other content in the URDF file will be ignored.
