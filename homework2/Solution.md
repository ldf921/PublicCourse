# Homework2

Homework by Liu, Dingfeng (2014011376).
## Problem 1
See the code and explaination is in the Ipython Notebook python/histogram.ipynb, for your convenience a HTML converted file is python/histogram.file.

## Problem 2
The point cloud without ground is pointclud.jpg. I simply manually find a threshold by looking at histogram, and take all points with height (z coordinate) less than the threshold as ground points.

## Problem 3
The code is rgb2gray.cc. Using opencv to decode a jpg image, and converted to grayscale by formula 0.299 R + 0.587 G + 0.114 B.

## Problem 4
See edge.jpg, code is edge_detect.cc. The method is first convert the image to grayscale and apply gaussian filter, finally using Canny edge detector in opencv.
Reference [OpenCV Tutorials-12](http://blog.csdn.net/poem_qianmo/article/details/25560901).