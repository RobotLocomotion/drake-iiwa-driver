# -*- python -*-

workspace(name = "drake_iiwa_driver")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

new_local_repository(
    name = "kuka_fri",
    path = "kuka-fri",
    build_file = "tools/kuka-fri.BUILD"
    )

(DRAKE_COMMIT, DRAKE_CHECKSUM) = (
    "e27fa1db17775ca3e141921b1570bde6beff448b",
    "f1508894c26db196506aa81970a9ba577b697a94b14137427f8e1d4c7e50cfb3",
)
# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
# DRAKE_CHECKSUM = "0" * 64

# Download a specific commit of Drake, from github.
http_archive(
    name = "drake",
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = "drake-{}".format(DRAKE_COMMIT),
    urls = [x.format(DRAKE_COMMIT) for x in [
        "https://github.com/RobotLocomotion/drake/archive/{}.tar.gz",
    ]],
)

load("@drake//tools/workspace/cc:repository.bzl", "cc_repository")
cc_repository(name = "cc")

load("@drake//tools/workspace/glib:repository.bzl", "glib_repository")
glib_repository(name = "glib")

load("@drake//tools/workspace/gflags:repository.bzl", "gflags_repository")
gflags_repository(name = "gflags")

load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
load("@drake//tools/workspace/lcm:repository.bzl", "lcm_repository")
lcm_repository(name = "lcm", mirrors = DEFAULT_MIRRORS)

load("@drake//tools/workspace/python:repository.bzl", "python_repository")
python_repository(name = "python")

load("@drake//tools/workspace/python3:repository.bzl", "python3_repository")
python3_repository(name = "python3")
