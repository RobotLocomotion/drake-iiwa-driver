# -*- python -*-

workspace(name = "drake_iiwa_driver")

new_local_repository(
    name = "kuka_fri",
    path = "kuka-fri",
    build_file = "tools/kuka-fri.BUILD"
    )

local_repository(
    name = "kythe",
    path = "tools/third_party/com_github_google_kythe",
)

load("@kythe//tools/build_rules/config:pkg_config.bzl", "pkg_config_package")

pkg_config_package(
    name = "glib",
    modname = "glib-2.0",
)

load("//tools:github.bzl", "github_archive")

github_archive(
    name = "drake",
    repository = "RobotLocomotion/drake",
    commit = "7965b0d0a14386d59c86b872a993cb92f0b9bedd",
    sha256 = "61eafa9d3eab0dfb4438f0cc0ec4b8b79d75b66287f31779ff4d127fba958baa",  # noqa
)

github_archive(
    name = "gflags",
    repository = "gflags/gflags",
    commit = "a69b2544d613b4bee404988710503720c487119a",
    sha256 = "8b3836d5ca34a2da4d6375cf5f2030c719b508ca16014fcc9d5e9b295b56a6c1",
)

github_archive(
    name = "lcm",
    repository = "lcm-proj/lcm",
    commit = "c0a0093a950fc83e12e8d5918a0319b590356e7e",
    sha256 = "f967e74e639ea56318242e93c77a15a504345c8200791cab70d9dad86aa969b2",  # noqa
    build_file = "tools/lcm.BUILD",
)
