# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:install_data.bzl", "install_data")
load("//tools:lint.bzl", "add_lint_tests")

drake_cc_library(
    name = "multi_pendulum_state_vector",
    srcs = ["gen/multi_pendulum_state_vector.cc"],
    hdrs = ["gen/multi_pendulum_state_vector.h"],
    deps = [
        "//drake/systems/framework:vector",
    ],
)

#drake_cc_library(
#    name = "multi_pendulum_lcm",
#    srcs = ["multi_pendulum_lcm.cc"],
#    hdrs = ["multi_pendulum_lcm.h"],
#    deps = [
#        ":multi_pendulum_state_vector",
#        "//drake/lcmtypes:multi_pendulum",
#        "//drake/systems/framework:leaf_system",
#        "//drake/systems/framework:vector",
#    ],
#)

#drake_cc_library(
#    name = "multi_pendulum_swing_up",
#    srcs = ["multi_pendulum_swing_up.cc"],
#    hdrs = ["multi_pendulum_swing_up.h"],
#    deps = [
#        ":multi_pendulum_plant",
#        "//drake/systems/trajectory_optimization:direct_collocation",
#    ],
#)

drake_cc_library(
    name = "multi_pendulum_plant",
    srcs = ["multi_pendulum_plant.cc"],
    hdrs = ["multi_pendulum_plant.h"],
    deps = [
        ":multi_pendulum_state_vector",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/systems/framework",
        "//drake/systems/sensors:rotary_encoders",
    ],
)

# Note: This is a development tool for testing LCM communication
# but does not actually provide test coverage as a stand-alone
# executable.
#drake_cc_binary(
#    name = "multi_pendulum_lcm_msg_generator",
#    testonly = 1,
#    srcs = ["test/multi_pendulum_lcm_msg_generator.cc"],
#    deps = [
#        "//drake/lcm",
#        "//drake/lcmtypes:multi_pendulum",
#        "//drake/systems/analysis:simulator",
#    ],
#)

#drake_cc_binary(
#    name = "multi_pendulum_run_lqr",
#    srcs = ["multi_pendulum_run_lqr.cc"],
#    add_test_rule = 1,
#    data = [":models"],
#    test_rule_args = ["-realtime_factor=0.0"],
#    deps = [
#        ":multi_pendulum_plant",
#        "//drake/common:find_resource",
#        "//drake/lcm",
#        "//drake/multibody:rigid_body_tree",
#        "//drake/multibody/parsers",
#        "//drake/multibody/rigid_body_plant:drake_visualizer",
#        "//drake/systems/analysis:simulator",
#        "@com_github_gflags_gflags//:gflags",
#    ],
#)

#drake_cc_binary(
#    name = "multi_pendulum_run_lqr_w_estimator",
#    testonly = 1,
#    srcs = ["multi_pendulum_run_lqr_w_estimator.cc"],
#    add_test_rule = 1,
#    data = [":models"],
#    test_rule_args = ["-realtime_factor=0.0"],
#    deps = [
#        ":multi_pendulum_plant",
#        "//drake/common:find_resource",
#        "//drake/common/proto:call_matlab",
#        "//drake/lcm",
#        "//drake/multibody:rigid_body_tree",
#        "//drake/multibody/joints",
#        "//drake/multibody/parsers",
#        "//drake/multibody/rigid_body_plant:drake_visualizer",
#        "//drake/systems/analysis:simulator",
#        "//drake/systems/estimators:kalman_filter",
#        "//drake/systems/framework:diagram",
#        "//drake/systems/primitives:linear_system",
#        "//drake/systems/primitives:signal_logger",
#        "//drake/systems/sensors:rotary_encoders",
#        "@com_github_gflags_gflags//:gflags",
#    ],
#)

drake_cc_binary(
    name = "multi_pendulum_run_passive",
    srcs = ["multi_pendulum_run_passive.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["-realtime_factor=0.0"],
    deps = [
        ":multi_pendulum_plant",
        "//drake/common:find_resource",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "@com_github_gflags_gflags//:gflags",
    ],
)

#drake_cc_binary(
#    name = "multi_pendulum_run_swing_up",
#    srcs = [
#        "multi_pendulum_run_swing_up.cc",
#        "multi_pendulum_spong_controller.h",
#    ],
#    data = [":models"],
#    deps = [
#        ":multi_pendulum_plant",
#        "//drake/common:find_resource",
#        "//drake/lcm",
#        "//drake/multibody:rigid_body_tree",
#        "//drake/multibody/joints",
#        "//drake/multibody/parsers",
#        "//drake/multibody/rigid_body_plant:drake_visualizer",
#        "//drake/systems/analysis",
#        "//drake/systems/controllers:linear_quadratic_regulator",
#        "@com_github_gflags_gflags//:gflags",
#    ],
#)

#drake_cc_binary(
#    name = "multi_pendulum_run_swing_up_traj_optimization",
#    srcs = ["multi_pendulum_run_swing_up_traj_optimization.cc"],
#    data = [":models"],
#    deps = [
#        ":multi_pendulum_plant",
#        ":multi_pendulum_swing_up",
#        "//drake/common:find_resource",
#        "//drake/lcm",
#        "//drake/multibody:rigid_body_tree",
#        "//drake/multibody/joints",
#        "//drake/multibody/parsers",
#        "//drake/multibody/rigid_body_plant:drake_visualizer",
#        "//drake/systems/analysis",
#        "//drake/systems/controllers:linear_quadratic_regulator",
#        "//drake/systems/primitives:trajectory_source",
#        "@com_github_gflags_gflags//:gflags",
#    ],
#)

#drake_cc_binary(
#    name = "multi_pendulum_spong_controller_w_lcm",
#    srcs = [
#        "multi_pendulum_spong_controller.h",
#        "multi_pendulum_spong_controller_w_lcm.cc",
#    ],
#    deps = [
#        ":multi_pendulum_lcm",
#        ":multi_pendulum_plant",
#        "//drake/lcm",
#        "//drake/multibody:rigid_body_tree",
#        "//drake/multibody/rigid_body_plant:drake_visualizer",
#        "//drake/systems/analysis",
#        "//drake/systems/controllers:linear_quadratic_regulator",
#    ],
#)

#drake_cc_binary(
#    name = "multi_pendulum_plant_w_lcm",
#    srcs = ["multi_pendulum_plant_w_lcm.cc"],
#    data = [":models"],
#    deps = [
#        ":multi_pendulum_lcm",
#        ":multi_pendulum_plant",
#        "//drake/common:find_resource",
#        "//drake/lcm",
#        "//drake/multibody/parsers",
#        "//drake/multibody/rigid_body_plant",
#        "//drake/multibody/rigid_body_plant:drake_visualizer",
#        "//drake/systems/analysis",
#        "//drake/systems/controllers:linear_quadratic_regulator",
#        "@com_github_gflags_gflags//:gflags",
#    ],
#)

#drake_cc_googletest(
#    name = "multi_pendulum_urdf_dynamics_test",
#    data = [":models"],
#    deps = [
#        ":multi_pendulum_plant",
#        "//drake/common:eigen_matrix_compare",
#        "//drake/common:find_resource",
#        "//drake/multibody/joints",
#        "//drake/multibody/parsers",
#        "//drake/multibody/rigid_body_plant",
#    ],
#)

install_data()

add_lint_tests()
