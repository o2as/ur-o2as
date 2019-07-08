# O2AS Graspability Estimation

This package provides object pose estimation based on graspability evaluation.

## How to use

### Estimation

``` sh
/root/catkin_ws/src/o2as_graspability_estiomation/connect-ros-and-matlab.sh
roslaunch o2as_graspability_estimation o2as_graspability_estimation.launch
rosservice call /search_grasp [part_id] [bin_id] [gripper] [update_image]
```

**NOTE:** Because of the algorithm, gripper-type 'inner' is limited parts usable.  
If apply to unusable parts, this service returns error below:

>ERROR: service [/search_grasp] responded with an error:  
>error processing request: local variable 'resp' referenced before assignment
