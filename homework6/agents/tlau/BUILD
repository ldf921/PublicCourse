package(default_visibility = ["//visibility:public"])

cc_library(
    name = "agent",
    hdrs = ["agent.h", "straight.h"],
    deps = [
        "//homework6/agents/tlau:control",
        "//homework6/simulation:vehicle_agent",
        "//homework6/simulation:vehicle_agent_factory",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "@glog",
        "@gflags//:gflags",
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
