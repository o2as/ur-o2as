# Introduction

This package offers the `FindObject` service, which takes in the camera, target object, expected pose and tolerance, and returns the object's pose.

# How to use

The VisionManager node advertises the `FindObject` service and passes it to the correct vision node. Each vision node is trained for a particular camera and knows all the models in the scene.

# Demo

Do:

    ```
    roslaunch o2as_vision demo.launch
    ```

This demonstration will:
- Launch the camera node (realsense sr300)  
- Save point cloud and image file captured by the camera node  
- Search an object in the image
- Add the detected object into the planning_scene of MoveIt  

Before executing the demo, you need to train o2as_cad_matching for the available object models + the camera that is used, by doing:

    ```
    roslaunch o2as_cad_matching train_assembly_realsense_************.launch
    ```

Replace the asterisks with the serial number of your realsense camera. You will need to add the parameters in `o2as_cad_matching/o2as_cad_matching/config/camera`, and the launch file in the `o2as_cad_matching` package.
