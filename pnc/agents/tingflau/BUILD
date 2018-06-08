package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tf_agent",
    hdrs = ["tf_agent.h"],
    deps = [
        "//pnc/agents/tingflau:control",
        "//pnc/agents/tingflau:route",
        "//pnc/simulation:vehicle_agent",
        "//pnc/simulation:vehicle_agent_factory",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "//common/proto:cc_vehicle_status_proto",
        "//common/proto:cc_route_proto",
        "@glog",
    ],
)

cc_library(
    name = "control",
    hdrs = ["control.h"],
    srcs = ["control.cc"],
    deps = [
        "@glog",
    ],
)

cc_library(
    name = "route",
    hdrs = ["route.h"],
    srcs = ["route.cc"],
    deps = [
        "//common/proto:cc_map_proto",
        "//common/utils/file",
        "//common/proto:cc_route_proto",
        "//pnc/map:map_lib",
        "@glog",
        "@eigen"
    ]
)