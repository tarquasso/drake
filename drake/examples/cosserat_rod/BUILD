# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:install_data.bzl", "install_data")
load("//tools:lint.bzl", "add_lint_tests")

drake_cc_library(
    name = "rod_element",
    srcs = [
        "rod_element.cc",
    ],
    hdrs = [
        "rod_element.h",
    ],
    deps = [
        "//drake/lcm",
        "//drake/lcmtypes:viewer",
        "//drake/multibody/multibody_tree",
    ],
)

#drake_cc_googletest(
#    name = "acrobot_multibody_plant_test",
#    deps = [
#        ":acrobot_multibody_plant",
#        "//drake/multibody/benchmarks/acrobot",
#    ],
#)

install_data()

add_lint_tests()
