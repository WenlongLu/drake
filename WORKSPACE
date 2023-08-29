# This file marks a workspace root for the Bazel build system.
# See `https://bazel.build/`.
workspace(name = "drake")

load("//tools/workspace:default.bzl", "add_default_workspace")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")


add_default_workspace()

http_archive(
    name = "com_github_nlohmann_json",
    sha256 = "ad2139f54927689c91a9312fcc1ec975732dcfc0a640d9a71463fdff629ee607",
    strip_prefix = "json-6af826d0bdb55e4b69e3ad817576745335f243ca",
    url = "https://github.com/nlohmann/json/archive/6af826d0bdb55e4b69e3ad817576745335f243ca.tar.gz",
)

# Add some special heuristic logic for using CLion with Drake.
load("//tools/clion:repository.bzl", "drake_clion_environment")

drake_clion_environment()

load("@bazel_skylib//lib:versions.bzl", "versions")

# This needs to be in WORKSPACE or a repository rule for native.bazel_version
# to actually be defined. The minimum_bazel_version value should match the
# version passed to the find_package(Bazel) call in the root CMakeLists.txt.
versions.check(minimum_bazel_version = "4.0")
