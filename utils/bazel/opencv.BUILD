licenses(["notice"])  # BSD 3-clause

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "core",
    srcs = glob(["lib/libopencv_*.so.3.4"]),
    hdrs = glob([
        "include/opencv2/core/*.h",
        "include/opencv2/core/*.hpp",
        "include/opencv2/core/hal/*.h",
        "include/opencv2/*.hpp",
        "include/opencv2/imgcodecs/*.h",
        "include/opencv2/videoio/*.h",
        "include/opencv2/flann/*.h",
        "include/opencv2/flann/*.hpp",
        "include/opencv2/dnn/*.hpp",  
        "include/opencv2/videostab/*.hpp",  
        "include/opencv2/video/*.hpp",
        "include/opencv2/video/*.h",
        "include/opencv2/calib3d/*.h",
        "include/opencv2/superres/*.hpp",
        "include/opencv2/stitching/*.hpp",  
        "include/opencv2/stitching/detail/*.hpp",
        "include/opencv2/shape/*.hpp",  
        "include/opencv2/photo/*.h",
    ]),
    includes = ["include"],
    linkstatic = 1,
)

cc_library(
    name = "calib3d",
    srcs = ["lib/x86_64-linux-gnu/libopencv_calib3d.so"],
    hdrs = glob([
        "include/opencv2/calib3d/*.h",
        "include/opencv2/calib3d/*.hpp",
    ]),
    includes = ["include"],
    linkstatic = 1,
)

cc_library(
    name = "contrib",
    srcs = ["lib/x86_64-linux-gnu/libopencv_contrib.so"],
    hdrs = glob([
        "include/opencv2/contrib/*.h",
        "include/opencv2/contrib/*.hpp",
    ]),
    includes = ["include"],
    linkstatic = 1,
)

cc_library(
    name = "features2d",
    srcs = ["lib/x86_64-linux-gnu/libopencv_features2d.so"],
    hdrs = glob([
        "include/opencv2/features2d/*.h",
        "include/opencv2/features2d/*.hpp",
    ]),
    includes = ["include"],
    linkstatic = 1,
)

cc_library(
    name = "highgui",
    srcs = ["lib/libopencv_highgui.so"],
    hdrs = glob([
        "include/opencv2/highgui/*.h",
        "include/opencv2/highgui/*.hpp",
    ]),
    linkstatic = 1,
)

cc_library(
    name = "imgproc",
    srcs = ["lib/libopencv_imgproc.so"],
    hdrs = glob([
        "include/opencv2/imgproc/*.h",
        "include/opencv2/imgproc/*.hpp",
    ]),
    includes = ["include"],
    linkstatic = 1,
)

cc_library(
    name = "ml",
    srcs = ["lib/libopencv_ml.so"],
    hdrs = glob([
        "include/opencv2/ml/*.h",
        "include/opencv2/ml/*.hpp",
    ]),
    includes = ["include"],
    linkstatic = 1,
)

cc_library(
    name = "objdetect",
    srcs = ["lib/libopencv_objdetect.so"],
    hdrs = glob([
        "include/opencv2/objdetect/*.h",
        "include/opencv2/objdetect/*.hpp",
    ]),
    includes = ["include"],
    linkstatic = 1,
)
