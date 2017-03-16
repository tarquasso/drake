# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

drake_cc_library(
    name = "soft_paddle_state_vector",
    srcs = ["soft_paddle_state_vector.cc"],
    hdrs = ["soft_paddle_state_vector.h"],
    deps = [
        "//drake/systems/framework:vector",
    ],
)

drake_cc_library(
    name = "soft_paddle_plant",
    srcs = ["soft_paddle_plant.cc"],
    hdrs = ["soft_paddle_plant.h"],
    deps = [
        ":soft_paddle_state_vector",
        "//drake/common:autodiff",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "soft_paddle_state_to_bot_visualizer",
    srcs = ["soft_paddle_state_to_bot_visualizer.cc"],
    hdrs = ["soft_paddle_state_to_bot_visualizer.h"],
    deps = [
        ":soft_paddle_plant",
    ],
)

drake_cc_binary(
    name = "soft_paddle_run_dynamics",
    srcs = [
        "soft_paddle_run_dynamics.cc",
    ],
    deps = [
        "@gflags//:gflags",
        ":soft_paddle_plant",
        ":soft_paddle_state_to_bot_visualizer",
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

drake_cc_library(
    name = "mirror_law_system",
    srcs = ["mirror_law_system.cc"],
    hdrs = ["mirror_law_system.h"],
    deps = [
        ":soft_paddle_plant",
        ":soft_paddle_state_to_bot_visualizer",
        "//drake/systems/primitives:first_order_low_pass_filter",
    ],
)

drake_cc_binary(
    name = "soft_paddle_mirror_law_test",
    srcs = [
        "soft_paddle_mirror_law_test.cc",
    ],
    deps = [
        ":mirror_law_system",
        ":soft_paddle_state_to_bot_visualizer",
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

drake_cc_library(
    name = "soft_paddle_poincare_map",
    srcs = ["soft_paddle_poincare_map.cc"],
    hdrs = ["soft_paddle_poincare_map.h"],
    deps = [
        ":mirror_law_system",
    ],
)

drake_cc_binary(
    name = "soft_paddle_poincare_map_test",
    srcs = [
        "soft_paddle_poincare_map_test.cc",
    ],
    deps = [
        ":soft_paddle_poincare_map",
        "//drake/math:discrete_algebraic_riccati_equation",
    ],
)

cpplint()
