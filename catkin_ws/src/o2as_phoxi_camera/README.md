o2as_phoxi_camera: ROS driver for PhoXi 3D Scanner
==================================================

This package provides a ROS driver for controlling [PhoXi 3D
scanners](http://www.photoneo.com/product-showcase/phoxi_3d_scanners). The
software is inspired by [ROS
drvier](https://github.com/photoneo/phoxi_camera) by the manufacturer,
i.e. [Photoneo co.](https://www.photoneo.com), however, its structure
is completely reorganized.

## Quick start

You have to invoke `PhoXiControl` in advance of running
`o2as_phoxi_camera`. `PhoXiControl` is a GUI-based controller for
PhoXi 3D scanners which can be freely downloaded from
[here](http://www.photoneo.com/download). It directly communicates
with one or more scanners. Our ROS driver, `o2as_phoxi_camera`,
establishes a connection to one of the available scanners, sends
control commands to it and receives various kinds of 3D data,
ex. point cloud, texture map, depth map, confidence map, etc., via
`PhoXiControl`. You can start `PhoXiControl` by typing;
```bash
$ PhoXiControl
```

`PhoXiControl` provides a virtual scanner device named
"InstalledExamples-PhoXi-example(File3DCamera)". Therefore, you can
test the ROS driver even if no actual scanner is connected to your
host. You can launch ROS driver and establish a connection to the
virtual scanner by typing as follows;

```bash
$ roslaunch o2as_phoxi_camera o2as_phoxi_camera_test.launch
```
If you wish to connect an actual device, you should specify its unique
ID;
```bash
$ roslaunch o2as_phoxi_camera o2as_phoxi_camera_test.launch id:='"1711015"'
```
The ID, "1711015" here, varies for each device. Please do not forget
to doubly enclose it with `'`s and `"`s because the ID should be treated
as a string.

By issueing the command above, a ROS viewer, `rviz`, will start as
well as the ROS driver,`o2as_phoxi_camera`. Here, `rviz` is configured
to subscribe two topics, `pointcloud` and `depth_map`, published by
`o2as_phoxi_camera`.  Then you should make the device enter into
acquisition mode;
```bash
$ rosservice call /o2as_phoxi_camera/start_acquisition
```
Now the device is ready for capturing and publishing 3D data. Please
type;
```bash
$ rosservice call /o2as_phoxi_camera/trigger_frame
$ rosservice call /o2as_phoxi_camera/get_frame 0
```
The observed point cloud and depth map will be displayed in
`rviz`. Repeating two commands above, you can obtain new data.

## Available ROS services

- **/o2as_phoxi_camera/start_acquisition** -- Make the scanner ready for image acquisition.
- **/o2as_phoxi_camera/stop_acquisition** -- Make the scanner stop image acquisition.
- **/o2as_phoxi_camera/trigger_frame** -- Capture 3D data.
- **/o2as_phoxi_camera/get_frame** -- Publish the captured data contained in a frame specified with a frame number. Currently, the frame number always should be 0.
- **/o2as_phoxi_camera/get_device_list** -- Enumerate all the PhoXi 3D scanner devices available including the virtual scanner.
- **/o2as_phoxi_camera/get_hardware_identification** -- Show unique ID of the scanner currently connected to.
- **/o2as_phoxi_camera/get_hardware_supported_capturing_modes** -- Enumerate all capturing modes supported by the scanner currently connected to.

## Published ROS topics

- **/o2as_phoxi_camera/confidence_map** -- A 2D map of values indicating reliability of 3D measurements at each pixel.
- **/o2as_phoxi_camera/depth_map** -- A 2D map of depth values, i.e. z-coordinate values of point cloud, in meters.
- **/o2as_phoxi_camera/normal_map** -- A 2D map of surface normals.
- **/o2as_phoxi_camera/texture** -- A 2D map of intensity values. The values are in 32bit floating point format and range from 0 to 4096.
- **/o2as_phoxi_camera/pointcloud** -- A 2D map of 3D points. Each 2D pixel has an associated intensity value in RGBA format as well as the corresponding 3D coordinates in meters.
- **/o2as_phoxi_camera/camera_info** -- Camera parameters including a 3x3 calibration matrix.

## ROS parameters

- **/o2as_phoxi_camera/resolution** -- Switch resolution.
- **/o2as_phoxi_camera/scan_multiplier**
- **/o2as_phoxi_camera/shutter_multiplier**
- **/o2as_phoxi_camera/trigger_mode** -- Switch between software trigger and free run modes
- **/o2as_phoxi_camera/timeout**
- **/o2as_phoxi_camera/confidence**
- **/o2as_phoxi_camera/send_pointcloud** -- Enable/disable publishing point cloud.
- **/o2as_phoxi_camera/send_normal_map** -- Enable/disable publishing normal maps.
- **/o2as_phoxi_camera/send_depth_map** -- Enable/disable publishing depth maps.
- **/o2as_phoxi_camera/send_confidence_map** -- Enable/disable publishing confidence maps.
- **/o2as_phoxi_camera/send_texture** -- Enable/disable publishing texture.
- **/o2as_phoxi_camera/intensity_scale** -- Change scale factor of intensity published in texture topic.
