#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/multi_pendulum/multi_pendulum_plant.h"
#include "drake/examples/multi_pendulum/gen/multi_pendulum_state_vector.h"
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
namespace multi_pendulum {
namespace {

// Simple example which simulates the (passive) multi_pendulum.  Run drake-visualizer
// to see the animated result.

DEFINE_double(realtime_factor, 1.0,
"Playback speed.  See documentation for "
"Simulator::set_target_realtime_rate() for details.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/multi_pendulum/MultiPendulum.urdf"),
      multibody::joints::kFixed, tree.get());

  systems::DiagramBuilder<double> builder;
  auto multi_pendulum = builder.AddSystem<MultiPendulumPlant>();
  multi_pendulum->set_name("multi_pendulum");
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  builder.Connect(multi_pendulum->get_output_port(0), publisher->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& multi_pendulum_context =
      diagram->GetMutableSubsystemContext(*multi_pendulum,
                                          simulator.get_mutable_context());

  double tau = 0;
  multi_pendulum_context.FixInputPort(0, Eigen::Matrix<double, 2, 1>::Constant(tau));

  // Set an initial condition that is sufficiently far from the downright fixed
  // point.
  MultiPendulumStateVector<double>* x0 = dynamic_cast<MultiPendulumStateVector<double>*>(
      multi_pendulum_context.get_mutable_continuous_state_vector());
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
}  // namespace multi_pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::multi_pendulum::do_main(argc, argv);
}
