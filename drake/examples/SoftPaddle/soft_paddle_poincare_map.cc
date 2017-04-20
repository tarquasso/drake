#include "drake/examples/SoftPaddle/soft_paddle_poincare_map.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#include <iomanip> //allowing to set precision of the PRINT_VAR outputs to terminal
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace examples {
namespace soft_paddle {

using systems::Context;
using systems::ContinuousState;
//using systems::DifferenceState;
using systems::SystemOutput;

template <typename T>
SoftPaddlePoincareMap<T>::SoftPaddlePoincareMap(
    double time_step, bool filter_commanded_angle) :
    time_step_(time_step), filter_commanded_angle_(filter_commanded_angle) {
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
void SoftPaddlePoincareMap<T>::ComputeNextState(
    const T &paddle_aim, const T &stroke_strength,
    const T &xn, const T &zn, const T &xdotn, T *xnext, T *znext, T *xdotnext) const {

  T dt = time_step_;
  const T zdotn = 0.0; //assume to slice at the peak

  auto paddle_plant =
      std::make_unique<SoftPaddleWithMirrorControl<T>>(
          paddle_aim, stroke_strength, filter_commanded_angle_);

  // Allocate workspace.
  auto paddle_context = paddle_plant->CreateDefaultContext();
  std::unique_ptr<ContinuousState<T>> derivs =
      paddle_plant->AllocateTimeDerivatives();

  paddle_plant->set_initial_conditions(paddle_context.get(), xn, zn, xdotn, zdotn);
  SoftPaddleStateVector<T>* xc_paddle =
      paddle_plant->GetMutablePlantStateVector(paddle_context.get());
  const systems::VectorBase<T>& paddle_xcdot =
      paddle_plant->GetSubsystemDerivatives(*derivs, &paddle_plant->get_soft_paddle_plant())->get_vector();

  // WARNING: this is the ENTIRE model derivatives (including filter).
  systems::VectorBase<T>* xcdot = derivs->get_mutable_vector();

  // Set initial conditions to be [xn, zn, xdotn, 0.0]. zdotn = 0.0, because we assume to be at the peak
  paddle_context->set_time(0.0);
  paddle_plant->set_initial_conditions(paddle_context.get(), xn, zn, xdotn, zdotn);
  //xc->SetFromVector(VectorX<T>::Zero(xc->size()));
  //xc->set_x(xn);
  //xc->set_z(zn);
  auto xc = paddle_context->get_mutable_continuous_state()->get_mutable_vector();

  // Previous time step solution.
  SoftPaddleStateVector<T> xc_paddle0; xc_paddle0.SetFromVector(xc_paddle->get_value());

  // Advance the paddle system using a simple explicit Euler scheme.
  int iteration = 0;
  do {
    iteration++;
    paddle_plant->CalcTimeDerivatives(*paddle_context, derivs.get());

    // Compute derivative and update configuration and velocity.
    // xc(t+h) = xc(t) + dt * xcdot(t, xc(t), u(t))
    xc->PlusEqScaled(dt, *xcdot);  // xc += dt * xcdot

    //PRINT_VAR(iteration);
    ///PRINT_VAR(xc_paddle0.zdot());
    //PRINT_VAR(xc_paddle->zdot());
    //PRINT_VAR(xc_paddle->x());

    // When going back up zdot crossed zero. Discard solution.
    if( xc_paddle0.zdot() > 0. && xc_paddle->zdot() < 0. ) break;

    paddle_context->set_time(paddle_context->get_time() + dt);
    xc_paddle0.SetFromVector(xc_paddle->get_value());
  }while(true);

  // Zero crossing time.
  //xcdot->zdot(); // WARNING!!: here xcdot is the ENTIRE model derivatives
  // (including filter) while xc_paddle only contains the paddle state.
  // QUESTION: is there a
  // After breaking from the loop above Context::get_time() has the time for
  // zdot > 0
  T t_zc = paddle_context->get_time() -
      xc_paddle0.zdot() / paddle_xcdot.GetAtIndex(3);
  // Computes time step that takes the solution to zdot = 0
  dt = t_zc - paddle_context->get_time();

  //PRINT_VAR(paddle_context->get_time());
  //PRINT_VAR(t_zc);
  //PRINT_VAR(dt);

  DRAKE_ASSERT(dt > 0.0);

  // Advances the solution to t_zc.
  xc->PlusEqScaled(dt, *xcdot);  // xc += dt * xcdot

  //std::cout << std::setprecision(9);
  //PRINT_VAR(xc_paddle->x());

  paddle_context->set_time(paddle_context->get_time() + dt);

  *xnext = xc_paddle->x();
  *znext = xc_paddle->z();
  *xdotnext = xc_paddle->xdot();
  //*xdotnext = 0.0;
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
//template class SoftPaddlePoincareMap<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class SoftPaddlePoincareMap<Eigen::AutoDiffScalar<Eigen::Vector3d>>;
template class SoftPaddlePoincareMap<Eigen::AutoDiffScalar<Vector5<double>>>;
//template class SoftPaddlePoincareMap<AutoDiffXd>;

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake