# Introduction

This package provides object detection and pose estimation function
using omron cad matching library. 

# How to use

## Licensing

See readme of omron_cad_matching package

## Train

If you have no model, you need to train before execute demo.

    ```
    roslaunch o2as_cad_matching train_assembly_***.launch
    ```

## Collect point cloud and image data from the camera

If you wand to add captured frame data to image directory, execute data collection.

    ```
    roslaunch o2as_cad_matching data_collection_phoxi.launch
    or 
    roslaunch o2as_cad_matching data_collection_realsense.launch
    ```

## Search

Once you finished training, run object detection demo.

    ```
    roslaunch o2as_cad_matching demo.launch
    ```

# How to customize

## Data structure

You need to put data files into config and data folder before execute launch.

    - o2as_cad_matching/data/setting : put train and search setting files here.
    - o2as_cad_matching/data/camera_info : put camera setting files here.
    - o2as_cad_matching/data/object_config : put object config files here.
    - o2as_cad_matching/data/model : folder to output trained model.
    - o2as_cad_matching/data/image : folder to save pcloud and image data when search object
    - o2as_parts_description/meshes : cad model should be located here. file format of cad models are .stl

    note: all cad models should be in the /meshes directory of o2as_parts_description package 

## Add parts

1. add cad files to o2as_parts_description/meshes
1. add parts entry to o2as_cad_matching/data/object_config/assembly_parts_list.xml
1. add object config files to o2as_cad_matching/data/object_config
1. run training

## Change camera

1. add camera info file to o2as_cad_matching/data/camera_info
1. add train_setting file and search_param file to o2as_cad_matching/data/setting
1. run training

# Service interfaces

## SelectCamera

You need to specify camera_name when camera to be used to take image for object detection is changed.
Search parmeters for the selected camera are set to the search_server.
This service must be called before SearchObjects or SearchObjects2

## SelectObject

You need to specify object_id to be detected when target object is changed.
trained model file for the selected object and the camera is loaded on the search_server.
This service must be called before SearchObjects or SearchObjects2

## SetSearchArea

You can set search area to reduce misdetection.
Objects outside the search area is ignored.
This service should be called before SearchObjects or SearchObjects2 if necessary.

## SetMaskImage

You can give mask image to limit search area of object detection.
Mask image should be specified by filename.
This service should be called before SearchObjects or SearchObjects2 if necessary.

## SetSearchRange

You can set minimum distance and maximum distance from the camera.
Object exists in range from min_dist to max_dist would be detected.
This service should be called before SearchObjects or SearchObjects2 if necessary.

## SearchObjects

This package provides two ways of search service via file and ros message.
File interface receives filenames of point cloud and texture image and mask image and returns SearchResult messsage.
ROS message interface receives sensor_msgs::PointCloud2 and sensor_msgs::Image and returns SearchResult messsage.
File interface is available without camera node so it is useful to test performance of cad matching using saved image.

## SearchObjects2

ROS message interface passes point cloud and image data via memory so it could be more efficient.
Note: unfortunately both phoxi camera and omron cad matching library works in millimeter unit and 
ros requires meter unit so twice unit conversion might causes performance issue.
