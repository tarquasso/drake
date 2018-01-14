#include <gtest/gtest.h>

#include "drake/multibody/multibody_tree/test/double_pendulum_3d_plant.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test {
namespace {

using systems::RungeKutta3Integrator;

GTEST_TEST(DoublePendulum3DTest, Trajectory) {
  // Testing parameters.
  const double kAccuracy = 1.0e-5;
  const double kMaxDt = 0.1;
  const double kEndTime = 10.0;
  const double kTolerance = 1.0e-10;

  // Physical parameters.
  double m1 = 1;
  double l1 = 1;
  double m2 = 1;
  double l2 = 1;
  double gravity = 9.81;

  // Create initial state, ensuring it's not too crazy so that the integrator
  // won't take forever.
  Vector6<double> position;
  Vector6<double> velocity;
  position << 0.1, 0.2, 0.3, -0.3, -0.2, -0.1;
  velocity << -0.01, 0.02, -0.03, 0.04, -0.05, 0.06;

  // Create two plants with different forward dynamics solvers.
  DoublePendulum3DPlant<double> MM_plant(
      m1, l1, m2, l2, gravity,
      DoublePendulum3DPlant<double>::ForwardSolver::MassMatrix);

  DoublePendulum3DPlant<double> AB_plant(
      m1, l1, m2, l2, gravity,
      DoublePendulum3DPlant<double>::ForwardSolver::ArticulatedBody);

  // Create simulation and set initial state.
  systems::Simulator<double> MM_simulator(MM_plant);
  systems::Context<double>& MM_context = MM_simulator.get_mutable_context();
  MM_plant.SetState(&MM_context, position, velocity);

  systems::Simulator<double> AB_simulator(AB_plant);
  systems::Context<double>& AB_context = AB_simulator.get_mutable_context();
  AB_plant.SetState(&AB_context, position, velocity);

  // Set up integrator and simulation.
  MM_simulator.Initialize();
  auto* MM_integrator =
      MM_simulator.reset_integrator<RungeKutta3Integrator<double>>(
          MM_plant, &MM_context);
  MM_integrator->set_maximum_step_size(kMaxDt);
  MM_integrator->set_target_accuracy(kAccuracy);

  AB_simulator.Initialize();
  auto* AB_integrator =
      AB_simulator.reset_integrator<RungeKutta3Integrator<double>>(
          AB_plant, &AB_context);
  AB_integrator->set_maximum_step_size(kMaxDt);
  AB_integrator->set_target_accuracy(kAccuracy);

  // Run simulation.
  MM_simulator.StepTo(kEndTime);
  AB_simulator.StepTo(kEndTime);

  // Get results.
  Vector6<double> MM_position, MM_velocity;
  Vector6<double> AB_position, AB_velocity;
  MM_plant.GetState(&MM_context, &MM_position, &MM_velocity);
  AB_plant.GetState(&AB_context, &AB_position, &AB_velocity);

  // Compare results.
  EXPECT_TRUE(CompareMatrices(
      AB_position, MM_position, kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(
      AB_velocity, MM_velocity, kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake