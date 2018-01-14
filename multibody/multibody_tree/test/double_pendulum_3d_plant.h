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

  void SetState(Context<T>* context,
                const Vector6<T> position,
                const Vector6<T> velocity);

  void GetState(const Context<T>* context,
                EigenPtr<VectorX<T>> position,
                EigenPtr<VectorX<T>> velocity);

 private:
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

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