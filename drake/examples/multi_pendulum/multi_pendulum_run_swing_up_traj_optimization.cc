// Generates a swing-up trajectory for multi_pendulum and displays the trajectory
// in DrakeVisualizer. Trajectory generation code is based on
// pendulum_swing_up.cc.

#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/multi_pendulum/multi_pendulum_plant.h"
#include "drake/examples/multi_pendulum/multi_pendulum_swing_up.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

using drake::solvers::SolutionResult;

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

namespace drake {
namespace examples {
namespace multi_pendulum {
namespace {

DEFINE_double(realtime_factor, 1.0,
"Playback speed.  See documentation for "
"Simulator::set_target_realtime_rate() for details.");

int do_main() {
  systems::DiagramBuilder<double> builder;

  auto multi_pendulum = std::make_unique<MultiPendulumPlant<double>>();

  const int kNumTimeSamples = 21;
  const double kTrajectoryTimeLowerBound = 0.1;
  const double kTrajectoryTimeUpperBound = 10.0;

  const Eigen::Vector4d x0(0, 0, 0, 0);
  //const Eigen::Vector4d xG(M_PI*4/9, M_PI*1/9, 0, 0);
  const Eigen::Vector4d xG(M_PI*4/9, M_PI*1/9, 0, 0);

  // Current limit for MIT's multi_pendulum is 7-9 Amps, according to Michael Posa.
  const double kTorqueLimit = 3.0;

  const Eigen::Vector2d umin(Eigen::Vector2d::Constant(-kTorqueLimit));
  const Eigen::Vector2d umax(Eigen::Vector2d::Constant(kTorqueLimit));

  auto context = multi_pendulum->CreateDefaultContext();

  systems::DircolTrajectoryOptimization dircol_traj(
      multi_pendulum.get(), *context, kNumTimeSamples, kTrajectoryTimeLowerBound,
      kTrajectoryTimeUpperBound);
  AddSwingUpTrajectoryParams(kNumTimeSamples, x0, xG,
      umin, umax, &dircol_traj);

  const double timespan_init = 4;
  auto traj_init_x =
      PiecewisePolynomialType::FirstOrderHold({0, timespan_init}, {x0, xG});
  SolutionResult result = dircol_traj.SolveTraj(
      timespan_init, PiecewisePolynomialType(), traj_init_x);
  if (result != SolutionResult::kSolutionFound) {
    std::cerr << "Result is an Error" << std::endl;
    return 1;
  }

  const PiecewisePolynomialTrajectory pp_xtraj =
      dircol_traj.ReconstructStateTrajectory();
  std::cout <<"End Time: " << pp_xtraj.get_end_time()<< std::endl;
  auto state_source = builder.AddSystem<systems::TrajectorySource>(pp_xtraj);

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/multi_pendulum/MultiPendulum.urdf"),
      multibody::joints::kFixed, tree.get());

  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);

  // By default, the simulator triggers a publish event at the end of each time
  // step of the integrator. However, since this system is only meant for
  // playback, there is no continuous state and the integrator does not even get
  // called. Therefore, we explicitly set the publish frequency for the
  // visualizer.
  publisher->set_publish_period(1.0 / 60.0);

  builder.Connect(state_source->get_output_port(),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.Initialize();
  simulator.StepTo(kTrajectoryTimeUpperBound);

  PRINT_VAR(simulator.get_context().get_time())
  PRINT_VAR(simulator.get_num_steps_taken())
  PRINT_VAR(simulator.get_integrator()->supports_error_estimation())
  PRINT_VAR(simulator.get_integrator()->get_fixed_step_mode())
  PRINT_VAR(simulator.get_integrator()->get_maximum_step_size())
  PRINT_VAR(simulator.get_integrator()->get_largest_step_size_taken())
  PRINT_VAR(simulator.get_integrator()->get_num_steps_taken())
  PRINT_VAR(simulator.get_integrator()->get_num_step_shrinkages_from_error_control())
  PRINT_VAR(simulator.get_integrator()->get_num_step_shrinkages_from_substep_failures())

  return 0;
}

}  // namespace
}  // namespace multi_pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multi_pendulum::do_main();
}
