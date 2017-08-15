#include "drake/examples/DoublePendulum/double_pendulum_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/rotary_encoders.h"

using std::sin;
using std::cos;

namespace drake {
namespace examples {
namespace double_pendulum {

namespace {
constexpr int kNumDOF = 2;  // theta1 + theta2.
}

template <typename T>
DoublePendulumPlant<T>::DoublePendulumPlant(double m1, double m2, double l1, double l2,
                              double lc1, double lc2, double Ic1, double Ic2,
                              double b1, double b2, double g)
    : systems::LeafSystem<T>(
    systems::SystemTypeTag<double_pendulum::DoublePendulumPlant>{}),
      m1_(m1),
      m2_(m2),
      l1_(l1),
      l2_(l2),
      lc1_(lc1),
      lc2_(lc2),
      Ic1_(Ic1),
      Ic2_(Ic2),
      b1_(b1),
      b2_(b2),
      g_(g) {
  this->DeclareInputPort(systems::kVectorValued, 2);
  this->DeclareVectorOutputPort(&DoublePendulumPlant::OutputState);
  static_assert(DoublePendulumStateVectorIndices::kNumCoordinates == kNumDOF * 2, "");
  this->DeclareContinuousState(
      DoublePendulumStateVector<T>(),
      kNumDOF /* num_q */,
      kNumDOF /* num_v */,
      0 /* num_z */);
}

template <typename T>
template <typename U>
DoublePendulumPlant<T>::DoublePendulumPlant(const DoublePendulumPlant<U>& other)
    : DoublePendulumPlant<T>(
    other.m1(),
    other.m2(),
    other.l1(),
    other.l2(),
    other.lc1(),
    other.lc2(),
    other.Ic1(),
    other.Ic2(),
    other.b1(),
    other.b2(),
    other.g()) {}

template <typename T>
std::unique_ptr<DoublePendulumPlant<T>> DoublePendulumPlant<T>::CreateDoublePendulumMIT() {
  return std::make_unique<DoublePendulumPlant<T>>(2.4367,   // m1
                                           0.6178,   // m2
                                           0.2563,   // l1
                                           0,        // l2
                                           1.6738,   // lc1
                                           1.5651,   // lc2
                                           -4.7443,  // Ic1
                                           -1.0068,  // Ic2
                                           0.0320,   // b1
                                           0.0413);  // b2
  // Parameters are identified in a way that torque has the unit of current
  // (Amps), in order to simplify the implementation of torque constraint on
  // motors. Therefore, numbers here do not carry physical meanings.
}

template <typename T>
void DoublePendulumPlant<T>::OutputState(const systems::Context<T>& context,
                                  DoublePendulumStateVector<T>* output) const {
  output->set_value(
      dynamic_cast<const DoublePendulumStateVector<T>&>(
          context.get_continuous_state_vector())
          .get_value());
}

template <typename T>
Matrix2<T> DoublePendulumPlant<T>::MatrixH(const DoublePendulumStateVector<T>& x) const {
  const T c2 = cos(x.theta2());

  const T h12 = I2_ + m2l1lc2_ * c2;
  Matrix2<T> H;
  H << I1_ + I2_ + m2_ * l1_ * l1_ + 2 * m2l1lc2_ * c2, h12, h12, I2_;
  return H;
}

template <typename T>
Vector2<T> DoublePendulumPlant<T>::VectorC(const DoublePendulumStateVector<T>& x) const {
  const T s1 = sin(x.theta1()), s2 = sin(x.theta2());
  const T s12 = sin(x.theta1() + x.theta2());

  Vector2<T> C;
  C << -2 * m2l1lc2_ * s2 * x.theta2dot() * x.theta1dot() +
      -m2l1lc2_ * s2 * x.theta2dot() * x.theta2dot(),
      m2l1lc2_ * s2 * x.theta1dot() * x.theta1dot();

  // Add in G terms.
  C(0) += g_ * m1_ * lc1_ * s1 + g_ * m2_ * (l1_ * s1 + lc2_ * s12);
  C(1) += g_ * m2_ * lc2_ * s12;

  // Damping terms.
  C(0) += b1_ * x.theta1dot();
  C(1) += b2_ * x.theta2dot();

  return C;
}

// Compute the actual physics.
template <typename T>
void DoublePendulumPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const DoublePendulumStateVector<T>& x = dynamic_cast<const DoublePendulumStateVector<T>&>(
      context.get_continuous_state_vector());
  const Vector2<T>& tau = this->EvalVectorInput(context, 0)->CopyToVector();
  //const T& tau = this->EvalVectorInput(context, 0)->GetAtIndex(0);

  Matrix2<T> H = MatrixH(x);
  Vector2<T> C = VectorC(x);
  Matrix2<T> B;  // input matrix, ToDo: Identity initilization
  B << 1.0,0.0,0.0,1.0;  // input matrix, ToDo: Identity initilization
  //Matrix2<T> B = Matrix2<T>::Identity();

  Vector4<T> xdot;
  xdot << x.theta1dot(), x.theta2dot(), H.inverse() * (B * tau - C);
  derivatives->SetFromVector(xdot);
}

template <typename T>
T DoublePendulumPlant<T>::DoCalcKineticEnergy(
    const systems::Context<T>& context) const {
  const DoublePendulumStateVector<T>& x = dynamic_cast<const DoublePendulumStateVector<T>&>(
      context.get_continuous_state_vector());

  Matrix2<T> H = MatrixH(x);
  Vector2<T> qdot(x.theta1dot(), x.theta2dot());

  return 0.5 * qdot.transpose() * H * qdot;
}

template <typename T>
T DoublePendulumPlant<T>::DoCalcPotentialEnergy(
    const systems::Context<T>& context) const {
  const DoublePendulumStateVector<T>& x = dynamic_cast<const DoublePendulumStateVector<T>&>(
      context.get_continuous_state_vector());

  using std::cos;
  const T c1 = cos(x.theta1());
  const T c12 = cos(x.theta1() + x.theta2());

  return -m1_ * g_ * lc1_ * c1 - m2_ * g_ * (l1_ * c1 + lc2_ * c12);
}

template class DoublePendulumPlant<double>;
template class DoublePendulumPlant<AutoDiffXd>;

template <typename T>
DoublePendulumWEncoder<T>::DoublePendulumWEncoder(bool double_pendulum_state_as_second_output) {
  systems::DiagramBuilder<T> builder;

  double_pendulum_plant_ = builder.template AddSystem<DoublePendulumPlant<T>>();
  double_pendulum_plant_->set_name("double_pendulum_plant");
  auto encoder =
      builder.template AddSystem<systems::sensors::RotaryEncoders<T>>(
          4, std::vector<int>{0, 1});
  encoder->set_name("encoder");
  builder.Cascade(*double_pendulum_plant_, *encoder);
  builder.ExportInput(double_pendulum_plant_->get_input_port(0));
  builder.ExportOutput(encoder->get_output_port());
  if (double_pendulum_state_as_second_output)
    builder.ExportOutput(double_pendulum_plant_->get_output_port(0));

  builder.BuildInto(this);
}

template <typename T>
DoublePendulumStateVector<T>* DoublePendulumWEncoder<T>::get_mutable_double_pendulum_state(
    systems::Context<T>* context) const {
  DoublePendulumStateVector<T>* x = dynamic_cast<DoublePendulumStateVector<T>*>(
      this->GetMutableSubsystemContext(*double_pendulum_plant_, context)
          .get_mutable_continuous_state_vector());
  DRAKE_DEMAND(x != nullptr);
  return x;
}

template class DoublePendulumWEncoder<double>;
template class DoublePendulumWEncoder<AutoDiffXd>;

std::unique_ptr<systems::AffineSystem<double>> BalancingLQRController(
    const DoublePendulumPlant<double>& double_pendulum) {
  auto context = double_pendulum.CreateDefaultContext();

  // Set nominal torque to zero.
  context->FixInputPort(0, Vector1d::Constant(0.0));

  // Set nominal state to the upright fixed point.
  DoublePendulumStateVector<double>* x = dynamic_cast<DoublePendulumStateVector<double>*>(
      context->get_mutable_continuous_state_vector());
  DRAKE_ASSERT(x != nullptr);
  x->set_theta1(M_PI);
  x->set_theta2(0.0);
  x->set_theta1dot(0.0);
  x->set_theta2dot(0.0);

  // Setup LQR Cost matrices (penalize position error 10x more than velocity
  // to roughly address difference in units, using sqrt(g/l) as the time
  // constant.
  Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
  Q(0, 0) = 10;
  Q(1, 1) = 10;
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();

  return systems::controllers::LinearQuadraticRegulator(
      double_pendulum, *context, Q, R);
}

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
