# Introduction

This is a realsense camera package customized for usage of team o2as. 
You can get point cloud, depth image and color image from sr300 camera 
 with specified serial_number.

this package provides services listed below.

GetFrame service
[request]
- bool publish                          # If true, publishes the point cloud over a ros topic
[response]
- sensor_msgs/Image color_image         # A synchronized color image.
- sensor_msgs/Image depth_image         # A synchronized depth image.
- sensor_msgs/PointCloud2 point_cloud   # A synchronized point cloud.

DumpFrame service
[request]
- string point_cloud_filename   # filename of the point cloud
- string depth_image_filename   # filename of the depth image
- string color_image_filename   # filename of the color image
[response]
- bool success

# Usage

1. Get image data via ros message

    ```
    roslaunch o2as_realsense_camera get_frame_demo.launch
    ```

    you can change args listed below in the launch file. 
    - node_name : "sr300" etc. 
    - serial_number : unique number of the camera device like 616205004841.
    - trigger_mode : if true, client needs to give trigger to the camera node to get frame.
    - send_color : if true, color image data is published.
    - send_depth : if true, depth image data is published.
    - send_cloud : if true, point cloud data is published.

1. Dump image for cad matching test

    ```
    roslaunch o2as_realsense_camera dump_frame_demo.launch
    ```

    point cloud and color image is saved into the file in the format acceptable by cad matching.
