# Homework4

## Problem 1
The code is `edge_detect.cc`. Usage
```
bazel run //homework4:edge_detect {pony_data_dir}/GigECameraDeviceTelephoto
```
It will detect edges in image x.jpg in that folder. x should be starting from 0.

## Problem 2
The coder is in `camera_lidar_fusion_utils.cc`. Two functions have been filled.

## Problem 3
The viewer is modify to generate the files and display only points inside each obstacle to visualize the results. After lanuching the window, press key 'g' to generate the perception obstacle files in pony data dir and save results to output_dir, 
press key 'n' to visualize the results.
```
bazel run //homework4:pointcloud_viewer_main -- --pony_data_dir {pony_data_dir} --output_dir {output_dir} --obstacle
```
The library for extracting points in perception obstacles is `perception_lib.h`, `perception_lib.cc`. The saved result is in homework4/results/x.label