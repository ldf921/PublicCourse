package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "opencv_example",
    srcs = ["opencv_example.cc"],
    deps = [
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgproc",
    ],
)

cc_binary(
    name = "distort",
    srcs = ["distort.cc"],
    deps = [
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgproc",
        "@eigen//:eigen"
    ],
)
