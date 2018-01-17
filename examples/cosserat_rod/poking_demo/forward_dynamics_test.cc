#include <gtest/gtest.h>

#include "drake/examples/cosserat_rod/poking_demo/cosserat_rod_plant.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace cosserat_rod {
namespace test {
namespace {

using drake::systems::ImplicitEulerIntegrator;
using drake::systems::DiagramBuilder;

GTEST_TEST(PokingDemoForwardDynamicsTest, Trajectory) {
  // Geometric parameters:
  const double length = 0.7;  // [m]
  const double radius1 = 0.05;
  const double radius2 = 0.02;

//  const double length = 2.0;  // [m]
//  const double radius1 = 0.005;
//  const double radius2 = 0.005;

  // Material parameters (aluminum):
  const double rho = 1000;  // [Kgr/m^3]
  const double E = 5.0e5;  // [Pa]
  const double nu = 0.5;  // Poission ratio [-]
  const double G = E / (2*(1+nu));  // Shear modulus. E = 2G(1+ν)
  //const double tau_d = 0.04469 / 10;  // [sec]
  double averageRadius = (radius1+radius2)/2;
  const double T1 = CosseratRodPlant<double>::EstimateTimeConstant(length,averageRadius,rho,E); //12.38;  // First period of oscillation.
  double zeta = 0.1;
  const double tau_d = CosseratRodPlant<double>::EstimateTau(T1,zeta); //T1 / 100;  // [sec]

  // Numerical parameters:
  const int num_elements = 20;
  const double dt = T1/1000;  // [sec]
  const double end_time = 5*T1;

  // Other derived numbers.
  const int num_spatial_dimensions = 3;

  // Mass matrix plant.
  systems::DiagramBuilder<double> MM_builder;

  auto MM_rod_plant = MM_builder.AddSystem<CosseratRodPlant>(
      length, radius1, radius2, rho,
      E, G, tau_d, tau_d, num_elements, num_spatial_dimensions,
      CosseratRodPlant<double>::Solver::MassMatrix);
  MM_rod_plant->set_name("MM Cosserat rod");

  auto MM_state_logger =
      MM_builder.AddSystem<systems::SignalLogger<double>>(
          MM_rod_plant->get_num_states());
  MM_state_logger->set_name("MM State Logger");
  MM_builder.Connect(MM_rod_plant->get_state_output_port(),
                  MM_state_logger->get_input_port());

  auto MM_diagram = MM_builder.Build();
  systems::Simulator<double> MM_simulator(*MM_diagram);
  systems::Context<double>& MM_rod_context =
      MM_diagram->GetMutableSubsystemContext(*MM_rod_plant,
                                             &MM_simulator.get_mutable_context());

  MM_rod_plant->SetBentState(&MM_rod_context);
  MM_simulator.Initialize();
  MM_simulator.set_publish_at_initialization(false);
  MM_simulator.set_publish_every_time_step(true);

  ImplicitEulerIntegrator<double>* MM_integrator =
      MM_simulator.reset_integrator<ImplicitEulerIntegrator<double>>(
          *MM_diagram, &MM_simulator.get_mutable_context());
  MM_integrator->set_fixed_step_mode(false);  // Good for steady state calculations.
  MM_integrator->set_maximum_step_size(dt);
  MM_integrator->set_target_accuracy(1.0e-3);
  
  MM_simulator.StepTo(end_time);

  // Articulated body plant.
  systems::DiagramBuilder<double> AB_builder;

  auto AB_rod_plant = AB_builder.AddSystem<CosseratRodPlant>(
      length, radius1, radius2, rho,
      E, G, tau_d, tau_d, num_elements, num_spatial_dimensions,
      CosseratRodPlant<double>::Solver::ArticulatedBody);
  AB_rod_plant->set_name("AB Cosserat rod");

  auto AB_state_logger =
      AB_builder.AddSystem<systems::SignalLogger<double>>(
          AB_rod_plant->get_num_states());
  AB_state_logger->set_name("AB State Logger");
  AB_builder.Connect(AB_rod_plant->get_state_output_port(),
                     AB_state_logger->get_input_port());

  auto AB_diagram = AB_builder.Build();
  systems::Simulator<double> AB_simulator(*AB_diagram);
  systems::Context<double>& AB_rod_context =
      AB_diagram->GetMutableSubsystemContext(*AB_rod_plant,
                                             &AB_simulator.get_mutable_context());

  AB_rod_plant->SetBentState(&AB_rod_context);
  AB_simulator.Initialize();
  AB_simulator.set_publish_at_initialization(false);
  AB_simulator.set_publish_every_time_step(true);

  ImplicitEulerIntegrator<double>* AB_integrator =
      AB_simulator.reset_integrator<ImplicitEulerIntegrator<double>>(
          *AB_diagram, &AB_simulator.get_mutable_context());
  AB_integrator->set_fixed_step_mode(false);  // Good for steady state calculations.
  AB_integrator->set_maximum_step_size(dt);
  AB_integrator->set_target_accuracy(1.0e-3);

  AB_simulator.StepTo(end_time);

  // Compare state logging results.
  MatrixX<double> MM_time_data(MM_state_logger->data().cols(),
                               MM_state_logger->data().rows());
  MatrixX<double> AB_time_data(AB_state_logger->data().cols(),
                               AB_state_logger->data().rows());

  const int MM_nsteps = MM_state_logger->sample_times().size();
  const int AB_nsteps = AB_state_logger->sample_times().size();

  MM_time_data = MM_state_logger->data().transpose();
  AB_time_data = AB_state_logger->data().transpose();

  const double kTolerance = 1.0e-3;
  ASSERT_TRUE(CompareMatrices(MM_time_data.row(MM_nsteps - 1),
                              AB_time_data.row(AB_nsteps - 1),
                              kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace test
}  // namespace cosserat_rod
}  // namespace examples
}  // namespace drake