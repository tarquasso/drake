#include "drake/examples/acrobot/acrobot_multibody_model.h"

#include <cmath>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

using std::sin;
using std::cos;

namespace drake {
namespace examples {
namespace acrobot {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

using namespace multibody;

namespace {
constexpr int kNumDOF = 2;  // theta1 + theta2.
}

template <typename T>
MultibodyAcrobotPlant<T>::MultibodyAcrobotPlant(double m1, double m2, double l1, double l2,
                              double lc1, double lc2, double Ic1, double Ic2,
                              double b1, double b2, double g)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<acrobot::MultibodyAcrobotPlant>{}),
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
  BuildMultibodyModeler();

  this->DeclareInputPort(systems::kVectorValued, 1);
  this->DeclareVectorOutputPort(&MultibodyAcrobotPlant::OutputState);
  static_assert(AcrobotStateVectorIndices::kNumCoordinates == kNumDOF * 2, "");
  this->DeclareContinuousState(
      AcrobotStateVector<T>(),
      kNumDOF /* num_q */,
      kNumDOF /* num_v */,
      0 /* num_z */);
}

template <typename T>
template <typename U>
MultibodyAcrobotPlant<T>::MultibodyAcrobotPlant(const MultibodyAcrobotPlant<U>& other)
    : MultibodyAcrobotPlant<T>(
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
void MultibodyAcrobotPlant<T>::BuildMultibodyModeler() {
  // Spatial inertia of the upper link about its frame L1 and expressed in L1.
  Vector3<double> com1_L1 =
      Vector3<double>::Zero();  // L1 is at the link's COM.
  // Rotational inertia of a thin rod along the y-axis.
  UnitInertia<double> G_L1 =
      UnitInertia<double>::ThinRod(Ic1_, Vector3<double>::UnitY());
  SpatialInertia<double> M_L1(m1_, com1_L1, G_L1);

  // Spatial inertia of the upper link about its frame L2 and expressed in L2.
  Vector3<double> com2_L2 =
      Vector3<double>::Zero();  // L2 is at the link's COM.
  // Rotational inertia of a thin rod along the y-axis.
  UnitInertia<double> G_L2 =
      UnitInertia<double>::ThinRod(Ic2_, Vector3<double>::UnitY());
  SpatialInertia<double> M_L2(m2_, com2_L2, G_L2);

  const Link<T>& world = modeler_.get_world_link();
  link1_ = &modeler_.template AddLink<RigidLink>("Link1", M_L1);
  link2_ = &modeler_.template AddLink<RigidLink>("Link2", M_L2);

  // Pose of the shoulder outboard frame So in link1's frame L1.
  const Isometry3d X_L1So{Translation3d(0.0, lc1(), 0.0)};

  // Shoulder joint.
  shoulder_ =
      &modeler_.template AddJoint<RevoluteJoint>(
          "ShoulderJoint",
          world, Isometry3d::Identity(), *link1_, X_L1So, Vector3d::UnitZ());

  // Elbow joint.
  // Pose of the elbow inboard frame Ei in the frame L1 of link1.
  const Isometry3d X_L1Ei{Translation3d(0.0, -lc1(), 0.0)};
  // Pose of the elbow outboard frame Eo in the frame L2 of link2.
  const Isometry3d X_L2Eo{Translation3d(0.0, lc2(), 0.0)};
  elbow_ =
      &modeler_.template AddJoint<RevoluteJoint>(
          "ElbowJoint",
          *link1_, X_L1Ei, *link2_, X_L2Eo, Vector3d::UnitZ());
  modeler_.Finalize();
}

#if 0
template <typename T>
std::unique_ptr<systems::LeafContext<T>>
MultibodyAcrobotPlant<T>::DoMakeContext() const {
  return modeler_.CreateDefaultContext();
}
#endif

template <typename T>
void MultibodyAcrobotPlant<T>::OutputState(const systems::Context<T>& context,
                                   AcrobotStateVector<T>* output) const {
  output->set_value(
      dynamic_cast<const AcrobotStateVector<T>&>(
          context.get_continuous_state_vector())
          .get_value());
}

template <typename T>
Matrix2<T> MultibodyAcrobotPlant<T>::MatrixH(const AcrobotStateVector<T>& x) const {
  const T c2 = cos(x.theta2());

  const T h12 = I2_ + m2l1lc2_ * c2;
  Matrix2<T> H;
  H << I1_ + I2_ + m2_ * l1_ * l1_ + 2 * m2l1lc2_ * c2, h12, h12, I2_;
  return H;
}

template <typename T>
Vector2<T> MultibodyAcrobotPlant<T>::VectorC(const AcrobotStateVector<T>& x) const {
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
void MultibodyAcrobotPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const AcrobotStateVector<T>& x = dynamic_cast<const AcrobotStateVector<T>&>(
      context.get_continuous_state_vector());
  const T& tau = this->EvalVectorInput(context, 0)->GetAtIndex(0);

  Matrix2<T> H = MatrixH(x);
  Vector2<T> C = VectorC(x);
  Vector2<T> B(0, 1);  // input matrix

  Vector4<T> xdot;
  xdot << x.theta1dot(), x.theta2dot(), H.inverse() * (B * tau - C);
  derivatives->SetFromVector(xdot);
}

template <typename T>
T MultibodyAcrobotPlant<T>::DoCalcKineticEnergy(
    const systems::Context<T>& context) const {
  const AcrobotStateVector<T>& x = dynamic_cast<const AcrobotStateVector<T>&>(
      context.get_continuous_state_vector());

  Matrix2<T> H = MatrixH(x);
  Vector2<T> qdot(x.theta1dot(), x.theta2dot());

  return 0.5 * qdot.transpose() * H * qdot;
}

template <typename T>
T MultibodyAcrobotPlant<T>::DoCalcPotentialEnergy(
    const systems::Context<T>& context) const {
  const AcrobotStateVector<T>& x = dynamic_cast<const AcrobotStateVector<T>&>(
      context.get_continuous_state_vector());

  using std::cos;
  const T c1 = cos(x.theta1());
  const T c12 = cos(x.theta1() + x.theta2());

  return -m1_ * g_ * lc1_ * c1 - m2_ * g_ * (l1_ * c1 + lc2_ * c12);
}

template class MultibodyAcrobotPlant<double>;
template class MultibodyAcrobotPlant<AutoDiffXd>;

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
