#include "drake/examples/SoftPaddle/soft_paddle_poincare_map.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace examples {
namespace soft_paddle {

using systems::Context;
using systems::ContinuousState;
//using systems::DifferenceState;
using systems::SystemOutput;

template <typename T>
SoftPaddlePoincareMap<T>::SoftPaddlePoincareMap() {
#if 0
    const int kSize = 2;  // The state includes [xn, zn].
    this->DeclareUpdatePeriodSec(1.0);   // Arbitrary sampling rate.

    // Inputs are the paddle aim and stroke strength.
    this->DeclareInputPort(
        systems::kVectorValued, kSize, systems::kDiscreteSampling);

    // Output is x[n+1].
    this->DeclareOutputPort(
        systems::kVectorValued, kSize, systems::kDiscreteSampling);

    this->DeclareDifferenceState(kSize);
#endif

  //paddle_plant_ = std::make_unique<SoftPaddleWithMirrorControl<T>>();
}

#if 0
template <typename T>
void SoftPaddlePoincareMap<T>::DoEvalDifferenceUpdates(
    const Context<T>& context,
    DifferenceState<T>* updates) const {
  const T paddle_aim = this->EvalVectorInput(context, 0)->GetAtIndex(0);
  const T stroke_strength = this->EvalVectorInput(context, 0)->GetAtIndex(1);

  T xn = context.get_difference_state(0)->GetAtIndex(0);
  T zn = context.get_difference_state(0)->GetAtIndex(1);

  T xnext, znext;
  ComputeNextSate(paddle_aim, stroke_strength, xn, zn, &xnext, &znext);

  updates->get_mutable_difference_state(0)->SetAtIndex(0, xnext);
  updates->get_mutable_difference_state(0)->SetAtIndex(1, znext);
}
#endif

template <typename T>
void SoftPaddlePoincareMap<T>::ComputeNextSate(
    const T& paddle_aim, const T& stroke_strength,
    const T& xn, const T& zn, T* xnext, T* znext) const {

  T dt = 1.0e-4;

  auto paddle_plant =
      std::make_unique<SoftPaddleWithMirrorControl<T>>(paddle_aim,
                                                       stroke_strength);

  // Allocate workspace.
  auto paddle_context = paddle_plant->CreateDefaultContext();
  std::unique_ptr<ContinuousState<T>> derivs =
      paddle_plant->AllocateTimeDerivatives();

  SoftPaddleStateVector<T>* xc =
      paddle_plant->GetMutablePlantStateVector(paddle_context.get());
  systems::VectorBase<T>* xcdot = derivs->get_mutable_vector();

  // Set initial conditions to be [xn, zn, 0.0, 0.0].
  paddle_context->set_time(0.0);
  paddle_plant->set_initial_conditions(paddle_context.get(), xn, zn);
  xc->SetFromVector(VectorX<T>::Zero(xc->size()));
  xc->set_x(xn);
  xc->set_z(zn);

  // Previous time step solution.
  SoftPaddleStateVector<T> xc0; xc0.SetFromVector(xc->get_value());

  // Advance the paddle system using a simple explicit Euler scheme.
  do {
    paddle_plant->CalcTimeDerivatives(*paddle_context, derivs.get());

    // Compute derivative and update configuration and velocity.
    // xc(t+h) = xc(t) + dt * xcdot(t, xc(t), u(t))
    xc->PlusEqScaled(dt, *xcdot);  // xc += dt * xcdot

    // When going back up zdot crossed zero. Discard solution.
    if( xc0.zdot() > 0. && xc->zdot() < 0. ) break;

    paddle_context->set_time(paddle_context->get_time() + dt);
    xc0.SetFromVector(xc->get_value());
  }while(true);

  // Zero crossing time.
  T t_zc = paddle_context->get_time() -
      xc0.zdot() / xcdot->GetAtIndex(3); //xcdot->zdot();
  // Computes time step that takes the solution to zdot = 0
  dt = t_zc - paddle_context->get_time();

  //PRINT_VAR(paddle_context->get_time());
  //PRINT_VAR(t_zc);
  //PRINT_VAR(dt);

  DRAKE_ASSERT(dt > 0.0);

  // Advances the solution to t_zc.
  xc->PlusEqScaled(dt, *xcdot);  // xc += dt * xcdot
  paddle_context->set_time(paddle_context->get_time() + dt);

  *xnext = xc->x();
  *znext = xc->z();
}

#if 0
template <typename T>
void SoftPaddlePoincareMap<T>::EvalOutput(
    const Context<T>& context, SystemOutput<T>* output) const {
output->GetMutableVectorData(0)->SetFromVector(
    context.get_difference_state(0)->CopyToVector());
}
#endif

template class SoftPaddlePoincareMap<double>;
// Eigen's tan fails at runtime if using AutoDiffXd.
// As a quick fix I am using a fixed size AutoDiffScalar.
template class SoftPaddlePoincareMap<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class SoftPaddlePoincareMap<AutoDiffXd>;

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake