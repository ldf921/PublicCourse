package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointcloud",
    srcs = ["pointcloud.cc"],
    hdrs = ["pointcloud.h"],
    deps = [
        "@eigen//:eigen",
        "@glog//:glog",
    ],
)

cc_binary(
    name = "main",
    srcs = ["main.cc"],
    deps = [
        ":pointcloud",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgproc",
    ],
)

cc_binary(
    name = "edge_detect",
    srcs = ["edge_detect.cc"],
    deps = [
        ":pointcloud",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgproc",
        "@eigen//:eigen",
    ],
)

cc_binary(
    name = "rgb2gray",
    srcs = ["rgb2gray.cc"],
    deps = [
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgproc",
    ],
)