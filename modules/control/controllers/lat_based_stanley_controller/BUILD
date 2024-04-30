load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_plugin(
    name = "liblat_stanley_controller.so",
    srcs = ["lat_stanley_controller.cc"],
    hdrs = ["lat_stanley_controller.h"],
    description = ":plugins.xml",
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/control_component/controller_task_base:control_task",
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_component/controller_task_base/common:trajectory_analyzer",
        "//modules/control/controllers/lat_based_stanley_controller/proto:lat_based_stanley_controller_conf_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
        "@com_google_absl//:absl",
    ],
)

apollo_package()
cpplint()