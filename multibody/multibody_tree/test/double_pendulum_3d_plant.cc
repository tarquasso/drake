#include "drake/multibody/multibody_tree/test/double_pendulum_3d_plant.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/system.h"
#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test {

template<typename T>
DoublePendulum3DPlant<T>::DoublePendulum3DPlant(
    double m1, double l1, double m2, double l2, double gravity,
    ForwardSolver solver)
    : m1_(m1), l1_(l1), m2_(m2), l2_(l2), gravity_(gravity), solver_(solver) {
  BuildMultibodyTreeModel();

  DRAKE_DEMAND(model_->get_num_positions() == 6);
  DRAKE_DEMAND(model_->get_num_velocities() == 6);
  DRAKE_DEMAND(model_->get_num_states() == 12);

  this->DeclareContinuousState(
      model_->get_num_positions(),
      model_->get_num_velocities(), 0);
}

template<typename T>
void DoublePendulum3DPlant<T>::SetState(
    systems::Context<T>* context, const Vector6<T> position,
    const Vector6<T> velocity) {
  shoulder_mobilizer_->set_angles(context, position.topRows(3));
  elbow_mobilizer_->set_angles(context, position.bottomRows(3));
  shoulder_mobilizer_->set_angular_velocity(context, velocity.topRows(3));
  elbow_mobilizer_->set_angular_velocity(context, velocity.bottomRows(3));
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
DoublePendulum3DPlant<T>::DoMakeContext() const {
  return model_->CreateDefaultContext();
}

template<typename T>
void DoublePendulum3DPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T> &context,
    systems::ContinuousState<T> *derivatives) const {
  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const int nv = model_->get_num_velocities();

  PositionKinematicsCache<T> pc(model_->get_topology());
  VelocityKinematicsCache<T> vc(model_->get_topology());
  model_->CalcPositionKinematicsCache(context, &pc);
  model_->CalcVelocityKinematicsCache(context, pc, &vc);

  std::vector<SpatialForce<T>> Fapplied_Bo_W_array(model_->get_num_bodies());
  VectorX<T> tau_applied_array(nv);
  model_->CalcForceElementsContribution(
      context, pc, vc, &Fapplied_Bo_W_array, &tau_applied_array);

  VectorX<T> qdot(nv);
  auto v = x.bottomRows(nv);
  model_->MapVelocityToQDot(context, v, &qdot);

  VectorX<T> qddot(nv);

  // Solve using the selected method.
  if (solver_ == ForwardSolver::MassMatrix) {
    MatrixX<T> M(nv, nv);
    model_->CalcMassMatrixViaInverseDynamics(context, &M);

    std::vector<SpatialAcceleration<T>> A_WB_array(model_->get_num_bodies());
    VectorX<T> vdot = VectorX<T>::Zero(nv);
    VectorX<T> C(nv);
    model_->CalcInverseDynamics(
        context, pc, vc, vdot, Fapplied_Bo_W_array, tau_applied_array,
        &A_WB_array, &Fapplied_Bo_W_array, &C);

    qddot << M.llt().solve(-C);
  } else {
    VectorX<T> vdot(nv);

    model_->CalcForwardDynamicsViaArticulatedBody(
        context, pc, vc, Fapplied_Bo_W_array, tau_applied_array, &vdot);

    const MultibodyTreeContext<T>* mbt_context =
        dynamic_cast<const MultibodyTreeContext<T>*>(&context);
    VectorX<T> q = mbt_context->get_positions();

    Vector3<T> qddot_shoulder;
    MapBallVDotToQDDot(q.head(3), qdot.head(3), vdot.head(3), &qddot_shoulder);
    Vector3<T> qddot_elbow;
    MapBallVDotToQDDot(q.tail(3), qdot.tail(3), vdot.tail(3), &qddot_elbow);

    qddot << qddot_shoulder, qddot_elbow;
  }

  VectorX<T> xdot(model_->get_num_states());
  xdot << qdot, qddot;
  derivatives->SetFromVector(xdot);
}

template<typename T>
void DoublePendulum3DPlant<T>::MapBallVDotToQDDot(
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& qdot,
    const Eigen::Ref<const VectorX<T>>& vdot,
    EigenPtr<VectorX<T>> qddot) const {
  using std::cos;
  using std::sin;
  using std::abs;

  const T cp = cos(q[1]);
  DRAKE_DEMAND(abs(cp) > 1.0e-3);

  const T sp = sin(q[1]);
  const T cy = cos(q[2]);
  const T sy = sin(q[2]);
  const T cpi = 1.0 / cp;

  const T r_dot = qdot[0];
  const T p_dot = qdot[1];
  const T y_dot = qdot[2];

  const T b0 = vdot[0];
  const T b1 = vdot[1];
  const T b2 = vdot[2];

  // Compute E⁻¹(q) * v̇.
  const T t = (cy * b0 + sy * b1) * cpi;
  Vector3<T> rhs = Vector3<T>(
      t,
      -sy * b0 + cy * b1,
      sp * t + b2);

  // Compute Ė⁻¹(q, q̇) * v.
  const T u = p_dot * cpi;
  Vector3<T> lhs = Vector3<T>(
      (r_dot * sp + y_dot) * u,
      -r_dot * y_dot * cp,
      (r_dot + y_dot * sp) * u);

  *qddot = lhs + rhs;
}

template<typename T>
void DoublePendulum3DPlant<T>::DoMapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    systems::VectorBase<T>* generalized_velocity) const {
  const int nq = model_->get_num_positions();
  const int nv = model_->get_num_velocities();

  DRAKE_ASSERT(qdot.size() == nq);
  DRAKE_DEMAND(generalized_velocity != nullptr);
  DRAKE_DEMAND(generalized_velocity->size() == nv);

  VectorX<T> v(nv);
  model_->MapQDotToVelocity(context, qdot, &v);
  generalized_velocity->SetFromVector(v);
}

template<typename T>
void DoublePendulum3DPlant<T>::DoMapVelocityToQDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    systems::VectorBase<T>* positions_derivative) const {
  const int nq = model_->get_num_positions();
  const int nv = model_->get_num_velocities();

  DRAKE_ASSERT(generalized_velocity.size() == nv);
  DRAKE_DEMAND(positions_derivative != nullptr);
  DRAKE_DEMAND(positions_derivative->size() == nq);

  VectorX<T> qdot(nq);
  model_->MapVelocityToQDot(context, generalized_velocity, &qdot);
  positions_derivative->SetFromVector(qdot);
}

template<typename T>
void DoublePendulum3DPlant<T>::BuildMultibodyTreeModel() {
  model_ = std::make_unique<MultibodyTree<T>>();
  DRAKE_DEMAND(model_ != nullptr);

  UnitInertia<double> G_Ucm =
      UnitInertia<double>::ThinRod(l1_, Vector3d::UnitY());
  SpatialInertia<double> M_Ucm(m1_, Vector3d::Zero(), G_Ucm);
  const RigidBody<T>& upper_link = model_->template AddBody<RigidBody>(M_Ucm);

  UnitInertia<double> G_Lcm =
      UnitInertia<double>::ThinRod(l2_, Vector3d::UnitY());
  SpatialInertia<double> M_Lcm(l2_, Vector3d::Zero(), G_Lcm);
  const RigidBody<T>& lower_link = model_->template AddBody<RigidBody>(M_Lcm);

  const Frame<T>& U_inboard_frame =
      model_->template AddFrame<FixedOffsetFrame>(
          model_->get_world_body(), Isometry3d::Identity());
  const Frame<T>& U_outboard_frame =
      model_->template AddFrame<FixedOffsetFrame>(
          upper_link, Isometry3d{Eigen::Translation3d(0.0, l1_ / 2, 0.0)});
  shoulder_mobilizer_ = &model_->template AddMobilizer<SpaceXYZMobilizer>(
      U_inboard_frame, U_outboard_frame);

  const Frame<T>& L_inboard_frame =
      model_->template AddFrame<FixedOffsetFrame>(
          upper_link, Isometry3d{Translation3d(0.0, -l1_ / 2, 0.0)});
  const Frame<T>& L_outboard_frame =
      model_->template AddFrame<FixedOffsetFrame>(
          lower_link, Isometry3d{Translation3d(0.0, l2_ / 2, 0.0)});
  elbow_mobilizer_ = &model_->template AddMobilizer<SpaceXYZMobilizer>(
      L_inboard_frame, L_outboard_frame);

  model_->template AddForceElement<UniformGravityFieldElement>(
      Vector3d(0.0, -gravity_, 0.0));

  model_->Finalize();
}

}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::multibody_tree::test::DoublePendulum3DPlant)