#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/drake_kuka_iiwa_robot.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {
namespace {

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Vector7d = Eigen::Matrix<double, 7, 1>;

const double kEpsilon = std::numeric_limits<double>::epsilon();

void TestKukaArmForwardDynamics(
    const Eigen::Ref<const VectorX<double>>& q,
    const Eigen::Ref<const VectorX<double>>& qdot) {
  // Create Kuka robot.
  const double gravity = 9.8;
  DrakeKukaIIwaRobot<double> kuka_robot(gravity);

  // Compute forward dynamics using ABA.
  Vector7d qddot;
  kuka_robot.CalcForwardDynamicsViaABA(q, qdot, &qddot);

  // Compute forward dynamics using explicit inverse.
  Vector7d qddot_expected;
  kuka_robot.CalcForwardDynamicsViaInverse(q, qdot, &qddot_expected);

  // Compare expected results against actual qddot.
  const double kTolerance = 50 * kEpsilon;
  EXPECT_TRUE(CompareMatrices(
      qddot, qddot_expected, kTolerance, MatrixCompareType::relative));
}

void BenchmarkKukaArmForwardDynamics(
    const Eigen::Ref<const VectorX<double>>& q,
    const Eigen::Ref<const VectorX<double>>& qdot,
    const int iterations) {
  // Create Kuka robot.
  const double gravity = 9.8;
  DrakeKukaIIwaRobot<double> kuka_robot(gravity);

  // Variable to hold forward dynamics output.
  Vector7d qddot;

  // Time ABA.
  auto aba_start = std::chrono::steady_clock::now();
  for (int i = 0; i < iterations; i++) {
    kuka_robot.CalcForwardDynamicsViaABA(q, qdot, &qddot);
  }
  auto aba_end = std::chrono::steady_clock::now();

  // Time explicit inverse.
  auto inverse_start = std::chrono::steady_clock::now();
  for (int i = 0; i < iterations; i++) {
    kuka_robot.CalcForwardDynamicsViaInverse(q, qdot, &qddot);
  }
  auto inverse_end = std::chrono::steady_clock::now();

  // Compute durations.
  std::chrono::duration<double> aba_diff = aba_end - aba_start;
  std::chrono::duration<double> inverse_diff = inverse_end - inverse_start;

  // Check expected performance ratio.
  const double ratio = 1.75;
  EXPECT_LE(ratio * aba_diff.count(), inverse_diff.count());
}

GTEST_TEST(KukaIIwaRobotKinematics, ForwardDynamicsTestA) {
  // State variables and helper angles.
  Vector7d q, qdot;
  double q30 = M_PI / 6, q45 = M_PI / 4, q60 = M_PI / 3;

  // Test 1: Static configuration.
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  qdot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  TestKukaArmForwardDynamics(q, qdot);

  // Test 2: Another static configuration.
  q << q30, -q45, q60, -q30, q45, -q60, q30;
  qdot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  TestKukaArmForwardDynamics(q, qdot);

  // Test 3: Non-static configuration.
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  qdot << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  TestKukaArmForwardDynamics(q, qdot);

  // Test 4: Another non-static configuration.
  q << -q45, q60, -q30, q45, -q60, q30, -q45;
  qdot << 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
  TestKukaArmForwardDynamics(q, qdot);
}

GTEST_TEST(KukaIIwaRobotKinematics, ForwardDynamicsTestB) {
  // State variables and helper angles.
  Vector7d q, qdot;
  double q30 = M_PI / 6, q45 = M_PI / 4, q60 = M_PI / 3;

  // Initialize a non-trivial state.
  q << q30, q45, q60, -q30, -q45, -q60, 0;
  qdot << 0.3, -0.1, 0.4, -0.1, 0.5, -0.9, 0.2;

  // Ensure solution validity.
  TestKukaArmForwardDynamics(q, qdot);

  // Ensure expected performance ratio.
  const int iterations = 1000;
  BenchmarkKukaArmForwardDynamics(q, qdot, iterations);
}

}  // namespace
}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake