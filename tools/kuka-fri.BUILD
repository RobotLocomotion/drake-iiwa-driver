# -*- python -*-

cc_library(
    name = "nanopb",
    srcs = [
        "src/nanopb-0.2.8/pb_decode.c",
        "src/nanopb-0.2.8/pb_encode.c",
    ],
    hdrs = [
        "src/nanopb-0.2.8/pb.h",
        "src/nanopb-0.2.8/pb_decode.h",
        "src/nanopb-0.2.8/pb_encode.h",
        "src/nanopb-0.2.8/pb_syshdr.h",
    ],
    copts = ["-w"],
    strip_include_prefix = "src/nanopb-0.2.8",
)

cc_library(
    name = "protobuf_gen",
    srcs = ["src/protobuf_gen/FRIMessages.pb.c"],
    hdrs = ["src/protobuf_gen/FRIMessages.pb.h"],
    copts = ["-w"],
    strip_include_prefix = "src/protobuf_gen",
    deps = [
        ":nanopb",
    ],
)

cc_library(
    name = "protobuf",
    srcs = glob([
        "src/protobuf/*.c",
        "src/protobuf/*.cpp",
    ]),
    hdrs = glob(["src/protobuf/*.h"]),
    copts = ["-w"],
    strip_include_prefix = "src/protobuf",
    deps = [
        ":nanopb",
        ":protobuf_gen",
    ],
)

cc_library(
    name = "clientbase_header",
    hdrs = glob(["src/base/*.h"]),
    copts = ["-w"],
    strip_include_prefix = "src/base",
    deps = [
        ":nanopb",
        ":protobuf",
        ":protobuf_gen",
    ],
)

cc_library(
    name = "kuka-fri-lib",
    srcs = glob([
        "src/base/*.cpp",
        "src/client_lbr/*.cpp",
        "src/connection/*.cpp",
    ]),
    hdrs = glob(["include/*.h"]),
    copts = [
        "-fpermissive",
        "-w",
    ],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
    deps = [
        ":clientbase_header",
        ":nanopb",
        ":protobuf",
        ":protobuf_gen",
    ],
)
