# -*- python -*-

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "kuka_driver",
    srcs = ["kuka_driver.cc",
    ],
    deps = [
        "@drake//drake/lcmtypes:iiwa",
        "@gflags//:gflags",
        "@kuka_fri//:kuka-fri-lib",
        "@lcm//:lcm",
        ]
    )
