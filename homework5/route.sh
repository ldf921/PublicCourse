#!/bin/sh
echo $1
bazel run //homework5:route_main -- --route_file_path $PWD/homework5/data/routes/route_request_$1.txt
bazel run //homework5:map_visualizer_main -- --route_file_path $PWD/homework5/data/routes/route_result_$1.txt