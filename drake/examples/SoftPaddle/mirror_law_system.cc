#include "drake/examples/SoftPaddle/mirror_law_system.h"

#include "drake/examples/SoftPaddle/soft_paddle_plant.h"
#include "drake/examples/SoftPaddle/soft_paddle_state_to_bot_visualizer.h"
#include "drake/examples/SoftPaddle/soft_paddle_state_vector.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/primitives/first_order_low_pass_filter-inl.h"
#include "drake/systems/primitives/constant_vector_source-inl.h"

namespace drake {
namespace examples {
namespace soft_paddle {

using systems::Context;
using systems::kVectorValued;
using systems::InputPortDescriptor;
using systems::OutputPortDescriptor;
using systems::System;
using systems::SystemOutput;
using systems::FirstOrderLowPassFilter;

template <typename T>
PaddleMirrorLawSystem<T>::PaddleMirrorLawSystem(
    const T& kappa_mirror, const T& kappa_energy,
    const T& kappa_rho,
    const T& kappa_rhodot) :
    kappa_mirror_(kappa_mirror),
    kappa_energy_(kappa_energy),
    kappa_rho_(kappa_rho),
    kappa_rhodot_(kappa_rhodot){
  // Input for the SoftPaddlePlant state.
  paddle_state_input_ =
      this->DeclareInputPort(
          kVectorValued,
          SoftPaddleStateVectorIndices::kNumCoordinates).get_index();

  // Input for law parameters.
  parameters_input_ =
      this->DeclareInputPort(kVectorValued, 2).get_index();

  // Output for the commanded paddle angle.
  this->DeclareOutputPort(kVectorValued, 1);
}

template <typename T>
const InputPortDescriptor<T>&
PaddleMirrorLawSystem<T>::get_parameters_input() const {
  return System<T>::get_input_port(parameters_input_);
}

template <typename T>
const OutputPortDescriptor<T>&
PaddleMirrorLawSystem<T>::get_paddle_angle_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
void PaddleMirrorLawSystem<T>::DoCalcOutput(const Context<T>& context,
                                            SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  auto parameters = this->EvalEigenVectorInput(context, parameters_input_);
  const T& kappa_mirror = parameters(0);
  const T& kappa_energy = parameters(1);
  const T& kappa_rho = parameters(2);
  const T& kappa_rhodot = parameters(3);

  (void) kappa_mirror; //why?
  (void) kappa_energy; //why?
  (void) kappa_rho; //why?
  (void) kappa_rhodot; //why this (void) operation?

  //const auto paddle_state = this->EvalVectorInput(context, 0);
  auto paddle_state =
      dynamic_cast<const SoftPaddleStateVector<T>*>(
          this->EvalVectorInput(context, paddle_state_input_));

  auto energy_delta = calculateEnergyDelta(paddle_state);

  // do inverse kinematics

  System<T>::GetMutableOutputVector(output, 0)(0) =
      - (kappa_mirror + kappa_energy * energy_delta) * paddle_state->zdot();

  //auto input_vector = this->EvalEigenVectorInput(context, 0);
 // System<T>::GetMutableOutputVector(output, 0) =
   //   k_.array() * input_vector.array();
}

template <typename T>
ApexMonitor<T>::ApexMonitor() {
  // Input for the SoftPaddlePlant state.
  paddle_state_input_port_ =
      this->DeclareInputPort(
          kVectorValued,
          SoftPaddleStateVectorIndices::kNumCoordinates).get_index();
  // Output for the commanded mirror law parameters.
  mirror_law_parameters_output_port_ =
      this->DeclareOutputPort(kVectorValued, 2).get_index();


  K_.resize(2, 3);
#if 0
  //x0 = 0.35, z0 = 0.4; Q = 1.0
  K_ << -1.9631795011057536, -0.0874935891761593,
         1.1414636533597844,  2.3218568669775732;

  //x0 = 0.35, z0 = 0.4; Q = 10.0
  K_ << -0.2327767144414060, -0.0378285950160658,
         0.1677343396027288,  0.3463513773374228;
#endif
  //x0 = 0.525, z0 = 0.4; Q = 10.0
  //K_ << 1.3676142316257427, -0.3461629657349821,
  //    -0.0359882941802672,  0.2011382989147669;
  //K_ << 1.3547103155711704, -0.2743088149682193,
  //    -0.0109592825187393,  0.0610306470571990;

  K_ <<  0.8901483990949484, -0.4811098290845131,  0.3292023609053129,
  0.0167561564042238,  0.1184014117644492,  0.0149988079185070;
  x0.resize(3);
  x0 << 0.418, 0.3, -0.0013725110828301;

  u0.resize(2);
  u0 << -0.1150299698129434, 0.0565732615072618;
}

template <typename T>
const InputPortDescriptor<T>&
ApexMonitor<T>::get_paddle_state_input() const {
  return System<T>::get_input_port(paddle_state_input_port_);
}

template <typename T>
const OutputPortDescriptor<T>&
ApexMonitor<T>::get_mirror_law_parameters_output() const {
  return System<T>::get_output_port(mirror_law_parameters_output_port_);
}

template <typename T>
void ApexMonitor<T>::DoCalcOutput(const Context<T>& context,
                                  SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  auto paddle_state =
      dynamic_cast<const SoftPaddleStateVector<T>*>(
          this->EvalVectorInput(context, paddle_state_input_port_));

  const T& w = paddle_state->zdot();

  // Fake initialization, we need state.
  if (w0_ == 0) {
    auto u = System<T>::GetMutableOutputVector(
        output, mirror_law_parameters_output_port_);
    u = u0;
  }

  if (w < 0 && w0_ >0) {
    std::cout << "Got to the Apex!" << std::endl;

    const T& xn = paddle_state->x();
    const T& zn = paddle_state->z();
    const T& xdotn = paddle_state->xdot();

    VectorX<T> x(x0.size());
    x << xn, zn, xdotn;

    auto u = System<T>::GetMutableOutputVector(
        output, mirror_law_parameters_output_port_);

    u = u0 - K_ * (x - x0);
    // In RSS version, no adjustment in apex monitor would be needed, just:
    // u0 = - K_ * x0

    std::cout << "u: " << u.transpose() << std::endl;
    std::cout << "x: " << x.transpose() << std::endl;
  }

  w0_ = w;

  //System<T>::GetMutableOutputVector(output, 0)(0) =
  //    kappa_mirror_ + kappa_energy_ * paddle_state->zdot();
  //auto input_vector = this->EvalEigenVectorInput(context, 0);
  // System<T>::GetMutableOutputVector(output, 0) =
  //   k_.array() * input_vector.array();
}

template <typename T>
SoftPaddleWithMirrorControl<T>::SoftPaddleWithMirrorControl(
    const T& kappa_mirror,
    const T& kappa_energy,
    const T& kappa_rho,
    const T& kappa_rhodot,
    bool filter_commanded_angle,
    bool with_lqr) {

  systems::DiagramBuilder<T> builder;

  auto mirror_system =
      builder.template AddSystem<PaddleMirrorLawSystem>(kappa_mirror, kappa_energy,
                                                        kappa_rho, kappa_rhodot);
  paddle_ = builder.template AddSystem<SoftPaddlePlant>();

  // Feedback loop.
  builder.Connect(paddle_->get_output_port(), mirror_system->get_input_port(0));
  if (filter_commanded_angle) {
    const double filter_time_constant = 0.15;  // In seconds.
    auto filter =
        builder.template AddSystem<FirstOrderLowPassFilter>(filter_time_constant);
    builder.Connect(
        mirror_system->get_paddle_angle_port(), filter->get_input_port());
    builder.Connect(
        filter->get_output_port(), paddle_->get_tau_port());
    command_angle_output_port_ =
        builder.ExportOutput(filter->get_output_port());
  } else {
    builder.Connect(
        mirror_system->get_paddle_angle_port(), paddle_->get_tau_port());
    builder.ExportOutput(mirror_system->get_paddle_angle_port());
  }

  state_output_port_ = builder.ExportOutput(paddle_->get_output_port());
  viz_elements_output_port_ =
      builder.ExportOutput(paddle_->get_elements_port());

  if (with_lqr) {
    auto apex_monitor =
        builder.template AddSystem<ApexMonitor>();
    builder.Connect(paddle_->get_output_port(),
                    apex_monitor->get_paddle_state_input());

    builder.Connect(apex_monitor->get_mirror_law_parameters_output(),
                    mirror_system->get_parameters_input());
  } else {
    auto constant_parameters_source =
        builder.template AddSystem<systems::ConstantVectorSource>(
            Vector2<T>(kappa_mirror, kappa_energy));
    builder.Connect(constant_parameters_source->get_output_port(),
                    mirror_system->get_parameters_input());
  }

  builder.BuildInto(this);
}

template <typename T>
const OutputPortDescriptor<T>&
SoftPaddleWithMirrorControl<T>::get_paddle_state_port() const {
  return System<T>::get_output_port(state_output_port_);
}

template <typename T>
const OutputPortDescriptor<T>&
SoftPaddleWithMirrorControl<T>::get_paddle_angle_port() const {
  return System<T>::get_output_port(command_angle_output_port_);
}

template <typename T>
const OutputPortDescriptor<T>&
SoftPaddleWithMirrorControl<T>::get_elements_port() const {
  return System<T>::get_output_port(viz_elements_output_port_);
}

template <typename T>
void SoftPaddleWithMirrorControl<T>::set_initial_conditions(
    Context<T>* context, const T& x0, const T& z0, const T& xdot0, const T& zdot0) const {
  Context<T>* paddle_context = this->GetMutableSubsystemContext(context,
                                                               paddle_);
  auto state =
      dynamic_cast<SoftPaddleStateVector<T>*>(
          paddle_context->get_mutable_continuous_state_vector());
  state->SetFromVector(VectorX<T>::Zero(state->size()));
  state->set_x(x0);
  state->set_z(z0);
  state->set_xdot(xdot0);
  state->set_zdot(zdot0);
}

template <typename T>
SoftPaddleStateVector<T>*
    SoftPaddleWithMirrorControl<T>::GetMutablePlantStateVector(
        Context<T>* context) const {
  Context<T>* paddle_context = this->GetMutableSubsystemContext(context,
                                                                paddle_);
  return dynamic_cast<SoftPaddleStateVector<T>*>(
      paddle_context->get_mutable_continuous_state_vector());
}

// Explicitly instantiates on the most common scalar types.
template class PaddleMirrorLawSystem<double>;
// Eigen's tan fails at runtime if using AutoDiffXd.
// As a quick fix I am using a fixed size AutoDiffScalar.
template class PaddleMirrorLawSystem<Eigen::AutoDiffScalar<Eigen::Vector3d>>;
template class PaddleMirrorLawSystem<Eigen::AutoDiffScalar<Vector5<double>>>;
//template class PaddleMirrorLawSystem<AutoDiffXd>;

template class SoftPaddleWithMirrorControl<double>;
// Eigen's tan fails at runtime if using AutoDiffXd.
// As a quick fix I am using a fixed size AutoDiffScalar.
template class SoftPaddleWithMirrorControl<
    Eigen::AutoDiffScalar<Eigen::Vector3d>>;
template class SoftPaddleWithMirrorControl<
    Eigen::AutoDiffScalar<Vector5<double>>>;
//template class SoftPaddleWithMirrorControl<AutoDiffXd>;

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
