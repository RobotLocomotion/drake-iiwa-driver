# -*- python -*-

workspace(name = "drake_iiwa_driver")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

new_local_repository(
    name = "kuka_fri",
    build_file = "tools/kuka-fri.BUILD.bazel",
    path = "kuka-fri",
)

(DRAKE_COMMIT, DRAKE_CHECKSUM) = (
    "v1.9.0",
    "a80a38d51bf2608489f7b44a882d00b76dac9765590afbfcf0c17090c16501ab",
)

DRAKE_STRIP_PREFIX = "drake-1.9.0"
# If using commit vs. a tag, uncomment below.
# DRAKE_STRIP_PREFIX = "drake-v{}".format(DRAKE_COMMIT)

# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
# DRAKE_CHECKSUM = "0" * 64

# Or to build against a local checkout of Drake, at the bash prompt set an
# environment variable before building:
#  export IIWA_LOCAL_DRAKE_PATH=/home/user/stuff/drake

# Load an environment variable.
load("//:environ.bzl", "environ_repository")

environ_repository(
    name = "environ",
    vars = ["IIWA_LOCAL_DRAKE_PATH"],
)

load("@environ//:environ.bzl", "IIWA_LOCAL_DRAKE_PATH")

# This declares the `@drake` repository as an http_archive from github,
# iff IIWA_LOCAL_DRAKE_PATH is unset.  When it is set, this declares a
# `@drake_ignored` package which is never referenced, and thus is ignored.
http_archive(
    name = "drake" if not IIWA_LOCAL_DRAKE_PATH else "drake_ignored",
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = DRAKE_STRIP_PREFIX,
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

load("@drake//tools/workspace:default.bzl", "add_default_workspace")

add_default_workspace()
