# Introduction

This package provides hand-eye calibration function

# How to use

## Licensing

This package uses 3D cad matching to detect calibration target marker object.
See readme of omron_cad_matching package.

## Online training

Run the commands below.
This moves robot and collect tf data for calibration and calculate calibration.
Collected transformations are saved into the csv file to offline debug. 

    ```
    roslaunch o2as_calibration robot_control.launch
    roslaunch o2as_calibration realsense_startup.launch
    roslaunch o2as_calibration realsense_data_collection.launch
    ```

Same for the phoxi (maybe)

    ```
    roslaunch o2as_calibration phoxi_calibration.launch
    roslaunch o2as_calibration phoxi_data_collection.launch
    ```

## Offline training

Calculate calibration using saved transformations.

    ```
    roslaunch o2as_cad_matching data_collection_phoxi.launch
    ```
