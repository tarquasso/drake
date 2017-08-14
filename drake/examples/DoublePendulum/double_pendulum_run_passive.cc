#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/examples/DoublePendulum/double_pendulum_plant.h"
#include "drake/examples/DoublePendulum/gen/double_pendulum_state_vector.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace double_pendulum {
namespace {

// Simple example which simulates the (passive) DoublePendulum.  Run drake-visualizer
// to see the animated result.

DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/DoublePendulum/DoublePendulum.urdf",
      multibody::joints::kFixed, tree.get());

  systems::DiagramBuilder<double> builder;
  auto double_pendulum = builder.AddSystem<DoublePendulumPlant>();
  double_pendulum->set_name("double_pendulum");
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  builder.Connect(double_pendulum->get_output_port(0), publisher->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>* double_pendulum_context =
      diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                          double_pendulum);

  double tau = 0;
  double_pendulum_context->FixInputPort(0, Eigen::Matrix<double, 1, 1>::Constant(tau));

  // Set an initial condition that is sufficiently far from the downright fixed
  // point.
  DoublePendulumStateVector<double>* x0 = dynamic_cast<DoublePendulumStateVector<double>*>(
      double_pendulum_context->get_mutable_continuous_state_vector());
  DRAKE_DEMAND(x0 != nullptr);
  x0->set_theta1(1.0);
  x0->set_theta2(1.0);
  x0->set_theta1dot(0.0);
  x0->set_theta2dot(0.0);

  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.Initialize();
  simulator.StepTo(10);
  return 0;
}

}  // namespace
}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::double_pendulum::do_main(argc, argv);
}
