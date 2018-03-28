# Homework2

Homework by Liu, Dingfeng (2014011376).
## Problem 1
See the code and explaination is in the Ipython Notebook python/histogram.ipynb, for your convenience a HTML converted file is python/histogram.file.

## Problem 2
The point cloud without ground is pointclud.jpg. I simply manually find a threshold by looking at histogram, and take all points with height (z coordinate) less than the threshold as ground points.

## Problem 3
The code is rgb2gray.cc. Using opencv to decode a jpg image, and converted to grayscale by formula 0.299 R + 0.587 G + 0.114 B.

## Problem 4
Usage 
```
 bazel run //homework2:edge_detect {path to data}/pony_data/GigECameraDeviceTelephoto
 bazel run //homework2:edge_detect {path to data}/pony_data/GigECameraDeviceWideangle
```
The problem will detect line boundary for all images in that folder, assume image is 0.jpg, 1.jpg, 2.jpg, ... 
The output is detect_0.jpg, detect_1.jpg, ... and lane_0.jpg, lane_1, ... in the same folder.
detect_*.jpg is the detection results, lane_*.jpg is a grayscale image only contain the mask of boundary lane.
See the result in sample_data/GigECameraDeviceTelephoto, sample_data/GigECameraDeviceWideangle as examples of detection results.

The method is using HoungeTransform to detect line on a Canny edge feature map. Then apply some handcrafted features to select those lines.
+ The boundary is brighter than background
+ The boundary lane contains two edges close to each other.
+ ...