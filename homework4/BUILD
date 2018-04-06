package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_lidar_fusion_utils",
    srcs = ["camera_lidar_fusion_utils.cc"],
    hdrs = ["camera_lidar_fusion_utils.h"],
    deps = [
        "//common/utils/math",
        "//homework2:pointcloud",
        "@eigen//:eigen",
        "@glog//:glog",
    ],
)

cc_library(
    name = "perception_lib",
    hdrs = ["perception_lib.h"],
    srcs = ["perception_lib.cc"],
    deps = [
        ":camera_lidar_fusion_utils",
        "//common/proto:cc_object_labeling_3d_proto",
        "//common/proto:cc_perception_proto",
        "//homework2:pointcloud",
        "@eigen//:eigen"
    ]
)


cc_binary(
    name = "perception_lib_main",
    srcs = ["perception_lib.cc"],
    deps = [
        ":camera_lidar_fusion_utils",
        ":perception_lib",
        "//common/proto:cc_object_labeling_3d_proto",
        "//common/proto:cc_perception_proto",
        "//homework2:pointcloud",
        "@eigen//:eigen"
    ]
)

cc_binary(
    name = "camera_lidar_fusion_main",
    srcs = ["camera_lidar_fusion_main.cc"],
    deps = [
        ":camera_lidar_fusion_utils",
        "//common/utils/display:color",
        "//common/utils/file:path",
        "//common/utils/strings:format",
        "@eigen//:eigen",
        "@glog//:glog",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:ml",
        "@opencv//:imgproc",
        "@opencv//:objdetect"
    ],
)

cc_library(
    name = "pointcloud_viewer",
    srcs = ["pointcloud_viewer.cc"],
    hdrs = ["pointcloud_viewer.h"],
    deps = [
        ":camera_lidar_fusion_utils",
        ":perception_lib",
        "//common/proto:cc_object_labeling_3d_proto",
        "//common/utils/display:painter_widget_base",
        "//common/utils/file:path",
        "//common/utils/strings:format",
        "@glog//:glog",
        "@qt//:qt_widgets",
    ],
)

cc_binary(
    name = "pointcloud_viewer_main",
    srcs = ["pointcloud_viewer_main.cc"],
    deps = [
        ":camera_lidar_fusion_utils",
        ":pointcloud_viewer",
        "//common/utils/display:painter_widget_base",
        "//common/utils/file:path",
        "//common/utils/strings:format",

        "@glog//:glog",
        "@qt//:qt_widgets",
    ],
)

cc_binary(
    name = "edge_detect",
    srcs = ["edge_detect.cc"],
    deps = [
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgproc",
        "@eigen//:eigen",
    ],
)
