# Homework 5

Liu, Dingfeng 2014011376

## Problem 2
The output file of the processed protobuf is `homework5/processed_map_proto.txt`. The libary is `homework5/map/map_lib.cc`, and the main function is `homework5/find_pred_succ.cc`.

## Problem 3
The output files are `homwork5/data/routes/route_result_x.txt`, x from 1 to 5. The library is `homework5/map/map_lib.h` and `homework5/map/map_lib.cc`. The main function is `homework5/route_main.cc`
The command line for fill in the routes is 
```
bazel run //homework5:route_main -- --route_file_path {working folder}/homework5/data/routes/route_request_{x}.txt

```