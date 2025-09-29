load("@rules_shell//shell:sh_test.bzl", "sh_test")

def _bazel_lint():
    files = native.glob(["*.bazel", "*.bzl"], allow_empty = True)
    if len(files) == 0:
        return
    buildifier = "@buildifier_prebuilt//:buildifier"
    locations = ["$(locations %s)" % f for f in files]
    sh_test(
        name = "buildifier_lint",
        srcs = [buildifier],
        data = files,
        args = ["-mode=check"] + locations,
        tags = ["buildifier_lint", "lint"],
    )

def add_lint_tests():
    _bazel_lint()
