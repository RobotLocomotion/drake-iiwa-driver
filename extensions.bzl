load("@bazel_tools//tools/build_defs/repo:local.bzl", "new_local_repository")

def _extensions_impl(module_ctx):
    new_local_repository(
        name = "kuka_fri",
        build_file = "//tools:kuka-fri.BUILD.bazel",
        path = "kuka-fri",
    )
    return module_ctx.extension_metadata(reproducible = True)

extensions = module_extension(
    implementation = _extensions_impl,
)
