# -*- python -*-

package(default_visibility = ["//visibility:public"])

genrule(
    name = "kuka-fri-build",
    srcs = ["build/GNUMake",
            "src",
            "include/friClientApplication.h",
            "include/friLBRClient.h",
            "include/friLBRState.h",
            "include/friClientIf.h",
            "include/friUdpConnection.h",
            "include/friLBRCommand.h",
            "include/friConnectionIf.h",
    ],
    local = 1,
    outs = [
        "libFRIClient.a",
        "friClientApplication.h",
        "friLBRClient.h",
        "friLBRState.h",
        "friClientIf.h",
        "friUdpConnection.h",
        "friLBRCommand.h",
        "friConnectionIf.h",
        ],
    cmd = "make -C external/kuka_fri/build/GNUMake &&" +
    "cp external/kuka_fri/lib/libFRIClient.a $(location libFRIClient.a) &&" +
    "cp $(location include/friClientApplication.h) $(location friClientApplication.h) &&" +
    "cp $(location include/friLBRClient.h) $(location friLBRClient.h) && " +
    "cp $(location include/friLBRState.h) $(location friLBRState.h) && " +
    "cp $(location include/friClientIf.h) $(location friClientIf.h) && " +
    "cp $(location include/friUdpConnection.h) $(location friUdpConnection.h) &&" +
    "cp $(location include/friLBRCommand.h) $(location friLBRCommand.h) && " +
    "cp $(location include/friConnectionIf.h) $(location friConnectionIf.h)"
    )

cc_library(
    name = "kuka-fri-lib",
    # This is just here to suppress a warning, it has no effect.
    linkstatic = 1,
    srcs = [
        "kuka-fri-build",
        ],
    deps =[],
)
