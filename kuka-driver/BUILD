# -*- python -*-

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "kuka_driver",
    srcs = [
        "kuka_driver.cc",
        "low_pass_filter.h"
    ],
    deps = [
        "@drake//drake/lcmtypes:iiwa",
        "@gflags//:gflags",
        "@kuka_fri//:kuka-fri-lib",
        "@lcm//:lcm",
    ],
)
