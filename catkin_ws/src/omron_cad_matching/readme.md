# Introduction

This package provides object detection and pose estimation functions using the OMRON CAD matching library. 
This is an internal package and best used through o2as_vision.
The o2as_vision package advertises the `FindObject` service, which is the main interface to this library.

The search service of this package receives the filename of a point cloud and color image, and 
returns a search result that includes information about the detected objects.
The OMRON CAD matching library works in millimeters internally, so the o2as_cad_matching layers converts the units.

## Service interface

This package provides two ways of search service via file and ros message.
File interface receives filenames of point cloud and texture image and mask image and returns SearchResult messsage.
ROS message interface receives sensor_msgs::PointCloud2 and sensor_msgs::Image and returns SearchResult messsage.
File interface is available without camera node so it is useful to test performance of cad matching using saved image.
ROS message interface passes point cloud and image data via memory so it could be more efficient.
Note: unfortunately both phoxi camera and omron cad matching library works in millimeter unit and 
ros requires meter unit so twice unit conversion might causes performance issue.

1. file interface (for test)
  [in]
  - string pcloud_filename
  - string image_filename
  - string mask_filename
  [out]
  - o2as_cad_matching/SearchResult search_result
  - bool success

1. ros message interface (for competitions)
  [in]
  - sensor_msgs/PointCloud2 pcloud
  - sensor_msgs/Image image
  - sensor_msgs/Image mask
  [out]
  - o2as_cad_matching/SearchResult search_result
  - bool success

# How to use

## Licensing

The Codemeter runtime must be running before executing the demo.

    ```
    CodeMeterLin
    ```

Cad matching library is protected using CodeMeter.
Please check the usb dongle is working on the host and the Codemeter runtime is running 
both on the host and in the docker container before use cad matching.

You need to install CodeMeter runtime to the host environment with the commands as follows.

    ```
    wget -O /tmp/codemeter.deb https://download.ensenso.com/s/ensensosdk/download?files=codemeter_6.40.2402.501_amd64.deb
    sudo dpkg -i /tmp/codemeter.deb
    sudo apt-get install -f -y
    sudo reboot
    ```

## Train

For each camera and 3D object, a series of particular parameters is necessary to perform the matching. The models can be generated using the following commands:

- One object(mesh) to one trained file

    ```
    roslaunch omron_cad_matching train_single.launch
    ```

- Multiple objects(meshes) to one trained file

    ```
    roslaunch omron_cad_matching train_multi.launch
    ```

- Multiple objects(meshes) to multiple trained files

    ```
    roslaunch omron_cad_matching train_multi_separate.launch
    ```

## Search

This package does not include access to the camera. You can test the
search using saved point cloud files and image files. 

- Search object in a test file using standalone node. Search result is saved into files:

    ```
    roslaunch omron_cad_matching search.launch
    ```

- Search object in a test file using search service. This launch files run two nodes: cad_matching_service_server and cad_matching_service_client.

    ```
    roslaunch omron_cad_matching search_single.launch
    ```

# How to customize

## Add parts

## Change camera

## Data structure

You need to put data files into config and data folder before run training.

    - data/camera_info : put camera setting files here.
    - data/object_config : put object config files here.
    - data/setting : put task specific setting files here.
    - data/meshes : cad model should be located here. file format of cad models are .stl
    - data/model : folder to output trained model.
    - data/image : folder to save pcloud and image data when search object

## Data preparation

    - global_search_param:
        - width: width of image. ex) 640
        - height: height of image. ex) 360
        - search_color_num: use 2d image or not for search. (0: not use, 1: use mono image, 3: use color image)
        - icp_color_num: use 2d image or not for position adjustment. (0: not use, 1: use mono image, 3: use color image) 
        - thresh_depth: threshold to calculate gradient of depth image. (1 to 1000, -1: auto)
        - thresh_grad: threshold to calculate gradient of 2d image. (1 to 1000, -1: auto)
        - thresh_inlier_pc: outlier judgement of 3d pointcloud. (0 to 359)
        - search_area: region of interest (left, top, right, bottom of processed region of the image)
        - max_result_num_all: maximum number of detected objects. (1 to 100)
        - thread_num: number of threads used to search.
    - object_search_param:
        - min_lat: minimum value of range of search around x axis (-90 to +90 deg)
        - max_lat: maximum value of range of search around x axis (-90 to +90 deg)
        - min_lon: minimum value of range of search around y axis (-90 to +90 deg)
        - max_lon: maximum value of range of search around y axis (-90 to +90 deg)
        - min_roll: minimum value of range of search around z axis (-180 to +180 deg)
        - max_roll: maximum value of range of search around z axis (-180 to +180 deg)
        - min_dist: minimum distance from 3d sensor to object (unit is mm)
        - max_dist: maximum distance from 3d sensor to object (unit is mm)
        - thresh_search: threshold of search
        - search_coef: coefficient of search candidate (0.1 to 10.0)
        - max_result_num: maximum number of detected objects. (1 to 100) 


