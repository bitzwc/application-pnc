load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "obstacle_avoid_parking_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libobstacle_avoid_parking.so",
    srcs = [
        "scenario_obstacle_avoid_parking.cc",
        "stage_obstacle_avoid.cc",
        "stage_parking.cc",
    ],
    hdrs = [
        "scenario_obstacle_avoid_parking.h",
        "stage_obstacle_avoid.h",
        "stage_parking.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/obstacle_avoid_parking/proto:obstacle_avoid_parking_proto",
    ],
)

apollo_package()

cpplint()