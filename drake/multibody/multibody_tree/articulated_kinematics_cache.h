#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/articulated_inertia.h"

namespace drake {
namespace multibody {

/// This class is one of the cache entries in MultibodyTreeContext. It holds the
/// kinematics results of computations relating to the Articulated Body Model.


template<typename T>
class ArticulatedKinematicsCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArticulatedKinematicsCache)

  explicit ArticulatedKinematicsCache(const MultibodyTreeTopology& topology) :
      num_nodes_(topology.get_num_bodies()) {
    Allocate();
  }

  const Matrix6X<T>& get_H_FM(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return H_FM_pool_[body_node_index];
  }

  Matrix6X<T>& get_mutable_H_FM(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return H_FM_pool_[body_node_index];
  }

  const ArticulatedInertia<T>& get_I_FMBo_B(BodyNodeIndex body_node_index)
  const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return I_FMBo_B_pool_[body_node_index];
  }

  ArticulatedInertia<T>& get_mutable_I_FMBo_B(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return I_FMBo_B_pool_[body_node_index];
  }

  const SpatialForce<T>& get_Fp_FMBo_B(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Fp_FMBo_B_pool_[body_node_index];
  }

  SpatialForce<T>& get_mutable_Fp_FMBo_B(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Fp_FMBo_B_pool_[body_node_index];
  }

  const SpatialAcceleration<T>& get_Az_FM(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Az_FM_pool_[body_node_index];
  }

  SpatialAcceleration<T>& get_mutable_Az_FM(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Az_FM_pool_[body_node_index];
  }

  const Matrix6X<T>& get_U_FM(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return U_FM_pool_[body_node_index];
  }

  Matrix6X<T>& get_mutable_U_FM(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return U_FM_pool_[body_node_index];
  }

  const MatrixX<T>& get_D_FM(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return D_FM_pool_[body_node_index];
  }

  MatrixX<T>& get_mutable_D_FM(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return D_FM_pool_[body_node_index];
  }

  const VectorX<T>& get_u_FM(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return u_FM_pool_[body_node_index];
  }

  VectorX<T>& get_mutable_u_FM(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return u_FM_pool_[body_node_index];
  }

 private:
  typedef std::vector<Matrix6X<T>> Matrix6X_PoolType;
  typedef std::vector<ArticulatedInertia<T>> ArticulatedInertia_PoolType;
  typedef std::vector<SpatialForce<T>> SpatialForce_PoolType;
  typedef std::vector<SpatialAcceleration<T>> SpatialAcceleration_PoolType;

  typedef std::vector<VectorX<T>> VectorX_PoolType;
  typedef std::vector<MatrixX<T>> MatrixX_PoolType;

  void Allocate() {
    H_FM_pool_.resize(static_cast<unsigned long>(num_nodes_));
    I_FMBo_B_pool_.resize(static_cast<unsigned long>(num_nodes_));
    Fp_FMBo_B_pool_.resize(static_cast<unsigned long>(num_nodes_));
    Az_FM_pool_.resize(static_cast<unsigned long>(num_nodes_));
    U_FM_pool_.resize(static_cast<unsigned long>(num_nodes_));
    D_FM_pool_.resize(static_cast<unsigned long>(num_nodes_));
    u_FM_pool_.resize(static_cast<unsigned long>(num_nodes_));
  }

  int num_nodes_{0};

  Matrix6X_PoolType H_FM_pool_{};
  ArticulatedInertia_PoolType I_FMBo_B_pool_{};
  SpatialForce_PoolType Fp_FMBo_B_pool_{};
  SpatialAcceleration_PoolType Az_FM_pool_{};
  Matrix6X_PoolType U_FM_pool_{};
  MatrixX_PoolType D_FM_pool_{};
  VectorX_PoolType u_FM_pool_{};
};

}  // namespace multibody
}  // namespace drake