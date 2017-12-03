#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

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

  const Matrix6<T>& get_IA_FMb_B(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return IA_FMb_B_pool_[body_node_index];
  }

  Matrix6<T>& get_mutable_IA_FMb_B(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return IA_FMb_B_pool_[body_node_index];
  }

  const Vector6<T>& get_FpA_FMb_B(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return FpA_FMb_B_pool_[body_node_index];
  }

  Vector6<T>& get_mutable_FpA_FMb_B(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return FpA_FMb_B_pool_[body_node_index];
  }

  const Vector6<T>& get_Az_FMb_B(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Az_FMb_B_pool_[body_node_index];
  }

  Vector6<T>& get_mutable_Az_FMb_B(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Az_FMb_B_pool_[body_node_index];
  }

  const Matrix6X<T>& get_U_FM_M(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return U_FM_M_pool_[body_node_index];
  }

  Matrix6X<T>& get_mutable_U_FM_M(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return U_FM_M_pool_[body_node_index];
  }

  const MatrixX<T>& get_D_FM_M(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return D_FM_M_pool_[body_node_index];
  }

  MatrixX<T>& get_mutable_D_FM_M(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return D_FM_M_pool_[body_node_index];
  }

  const VectorX<T>& get_u_FM_M(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return u_FM_M_pool_[body_node_index];
  }

  VectorX<T>& get_mutable_u_FM_M(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return u_FM_M_pool_[body_node_index];
  }

  const Vector6<T>& get_A_WB_B(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return A_WB_B_pool_[body_node_index];
  }

  Vector6<T>& get_mutable_A_WB_B(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return A_WB_B_pool_[body_node_index];
  }

 private:
  typedef std::vector<Matrix6<T>> Matrix6_PoolType;
  typedef std::vector<Matrix6X<T>> Matrix6X_PoolType;
  typedef std::vector<Vector6<T>> Vector6_PoolType;
  typedef std::vector<VectorX<T>> VectorX_PoolType;
  typedef std::vector<MatrixX<T>> MatrixX_PoolType;

  void Allocate() {
    IA_FMb_B_pool_.resize(static_cast<unsigned long>(num_nodes_));
    FpA_FMb_B_pool_.resize(static_cast<unsigned long>(num_nodes_));
    Az_FMb_B_pool_.resize(static_cast<unsigned long>(num_nodes_));
    U_FM_M_pool_.resize(static_cast<unsigned long>(num_nodes_));
    D_FM_M_pool_.resize(static_cast<unsigned long>(num_nodes_));
    u_FM_M_pool_.resize(static_cast<unsigned long>(num_nodes_));
    A_WB_B_pool_.resize(static_cast<unsigned long>(num_nodes_));

    // Spatial acceleration of world is zero.
    A_WB_B_pool_[world_index()] = Vector6<T>::Zero();
  }

  int num_nodes_{0};

  // Pool names are directly from [Jain 2010, Algorithm 7.2].
  Matrix6_PoolType IA_FMb_B_pool_{};
  Vector6_PoolType FpA_FMb_B_pool_{};
  Vector6_PoolType Az_FMb_B_pool_{};
  Matrix6X_PoolType U_FM_M_pool_{};
  MatrixX_PoolType D_FM_M_pool_{};
  VectorX_PoolType u_FM_M_pool_{};
  Vector6_PoolType A_WB_B_pool_{};
};

}  // namespace multibody
}  // namespace drake