#pragma once

#include "drake/multibody/multibody_tree/space_xyz_mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test {

using Eigen::Vector3d;
using Eigen::Translation3d;
using Eigen::Isometry3d;
using std::make_unique;
using std::unique_ptr;
using std::pow;
using systems::Context;

template<typename T>
class DoublePendulum3DPlant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DoublePendulum3DPlant)

  enum ForwardSolver {
    MassMatrix,
    ArticulatedBody
  };

  DoublePendulum3DPlant(
      double m1, double l1, double m2, double l2, double gravity,
      ForwardSolver solver);

  void SetState(Context<T>* context, const Vector6<T> position,
                const Vector6<T> velocity);

 private:
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  // This is a method to map v̇ to q̈ for SpaceXYZMobilizer. It is currently
  // implemented here but should be moved into the Mobilizer class. It is
  // required for the computation of forward dynamics using the articulated
  // body algorithm, which outputs v̇. For simple mobilizers where v̇ = q̈, this
  // is not necessary.
  //
  // Below is a brief description of the mathematical derivation of the
  // conversion. SpaceXYZMobilizer uses Euler Angle Sequence (1,2,3),
  // corresponding to roll (r), pitch (p), and yaw (y). We have the relation
  // described in [Diebel 2006, §5.2]: v = E(q) * q̇ => q̇ = E⁻¹(q) * v. By
  // taking the derivative of both sides and applying the chin rule, we reach
  // q̈ = Ė⁻¹(q, q̇) * v + E⁻¹(q) * v̇. The quantity on the right is easy to
  // evaluate directly from the given inputs. We can simplify the quantity on
  // the left by realizing that Ė⁻¹(q, q̇) * v = Ė⁻¹(q, q̇) * (E(q) * q̇). This
  // leads to a much simpler expression, which is compute using Sympy.
  //
  // >>> from sympy import *
  // >>> t = Symbol('t')
  // >>> r = Function('r')(t); p = Function('p')(t); y = Function('y')(t)
  // >>> E = Matrix([
  //   [cos(y) * cos(p), -sin(y), 0],
  //   [sin(y) * cos(p), cos(y), 0],
  //   [-sin(p), 0, 1]
  // ])
  // >>> Einv = Matrix([
  //   [cos(y) / cos(p), sin(y) / cos(p), 0],
  //   [-sin(y), cos(y), 0],
  //   [sin(p) * cos(y) / cos(p), sin(p) * sin(y) / cos(p), 1]
  // ])
  // >>> assert simplify(expand(E.inv())) == simplify(expand(Einv))
  // >>> rdot, pdot, ydot = symbols('\dot{r} \dot{p} \dot{y}')
  // >>> qdot = Matrix([[rdot], [pdot], [ydot]])
  // >>> qdot_subs = [(r.diff(t), rdot), (p.diff(t), pdot), (y.diff(t), ydot)]
  // >>> simplify(expand(Einv.diff(t).subs(qdot_subs) * E * qdot))
  //
  // This results in the expressions below for q̈ := [q̈0, q̈1, q̈2].
  // q̈0 = ṗ * (ṙ * sin(p) + ẏ) / cos(p)
  // q̈1 = -ṙ * ẏ * cos(p)
  // q̈2 = ṗ * (ṙ + ẏ * sin(p)) / cos(p)
  //
  // [Diebel 2006] Representing attitude: Euler angles, unit quaternions, and
  //               rotation vectors. Stanford University.
  void MapBallVDotToQDDot(
      const Eigen::Ref<const VectorX<T>>& q,
      const Eigen::Ref<const VectorX<T>>& qdot,
      const Eigen::Ref<const VectorX<T>>& vdot,
      EigenPtr<VectorX<T>> qddot) const;

  void DoCalcTimeDerivatives(
      const systems::Context<T> &context,
      systems::ContinuousState<T> *derivatives) const override;

  void DoMapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      systems::VectorBase<T>* generalized_velocity) const override;

  void DoMapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      systems::VectorBase<T>* qdot) const override;

  void BuildMultibodyTreeModel();

  const double m1_;
  const double l1_;
  const double m2_;
  const double l2_;
  const double gravity_;

  const ForwardSolver solver_;

  const SpaceXYZMobilizer<T>* shoulder_mobilizer_{nullptr};
  const SpaceXYZMobilizer<T>* elbow_mobilizer_{nullptr};

  std::unique_ptr<drake::multibody::MultibodyTree<T>> model_;
};

}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake