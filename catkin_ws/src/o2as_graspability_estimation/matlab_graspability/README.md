# matlab_graspability

This package offers algorithm and support functions to search graspable point of target parts from depth image.
These are written in MATLAB language.

## extractRoI

This script offers making regions of interest in depth image.

1. Take depth image using depth sensor.
2. Open the script `extractRoI.m` and change `nam` value to image's file path (line 17).
3. Press `Run` button on EDITOR tab, wait a second until drawing figure.
4. Select vertices by mouse click to make bin regions on the left upper bin.
5. Repeat for all bins.
