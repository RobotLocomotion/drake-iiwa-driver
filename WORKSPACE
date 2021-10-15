# -*- python -*-

workspace(name = "drake_iiwa_driver")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

new_local_repository(
    name = "kuka_fri",
    path = "kuka-fri",
    build_file = "tools/kuka-fri.BUILD"
    )

(DRAKE_COMMIT, DRAKE_CHECKSUM) = (
    "b79928f03f94780ef8096a08c109d85cefe428f8",
    "ac001ccac82d560b32038585129192e2931857d142db8989116450ec36f3f58f",
)
# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
# DRAKE_CHECKSUM = "0" * 64

# Or to build against a local checkout of Drake, at the bash prompt set an
# environment variable before building:
#  export IIWA_LOCAL_DRAKE_PATH=/home/user/stuff/drake

# Load an environment variable.
load("//:environ.bzl", "environ_repository")
environ_repository(name = "environ", vars = ["IIWA_LOCAL_DRAKE_PATH"])
load("@environ//:environ.bzl", IIWA_LOCAL_DRAKE_PATH = "IIWA_LOCAL_DRAKE_PATH")

# This declares the `@drake` repository as an http_archive from github,
# iff IIWA_LOCAL_DRAKE_PATH is unset.  When it is set, this declares a
# `@drake_ignored` package which is never referenced, and thus is ignored.
http_archive(
    name = "drake" if not IIWA_LOCAL_DRAKE_PATH else "drake_ignored",
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = "drake-{}".format(DRAKE_COMMIT),
    urls = [x.format(DRAKE_COMMIT) for x in [
        "https://github.com/RobotLocomotion/drake/archive/{}.tar.gz",
    ]],
)

# This declares the `@drake` repository as a local directory,
# iff IIWA_LOCAL_DRAKE_PATH is set.  When it is unset, this declares a
# `@drake_ignored` package which is never referenced, and thus is ignored.
local_repository(
    name = "drake" if IIWA_LOCAL_DRAKE_PATH else "drake_ignored",
    path = IIWA_LOCAL_DRAKE_PATH,
)
print("Using IIWA_LOCAL_DRAKE_PATH={}".format(IIWA_LOCAL_DRAKE_PATH)) if IIWA_LOCAL_DRAKE_PATH else None  # noqa


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

load("@drake//tools/workspace/rules_python:repository.bzl", "rules_python_repository")  # noqa
rules_python_repository(name = "rules_python", mirrors = DEFAULT_MIRRORS)
