#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/articulated_body_inertia.h"

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

  const ArticulatedBodyInertia<T>& get_P_PB_W(BodyNodeIndex body_node_index)
  const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return P_PB_W_pool_[body_node_index];
  }

  ArticulatedBodyInertia<T>& get_mutable_P_PB_W(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return P_PB_W_pool_[body_node_index];
  }

  const SpatialForce<T>& get_Fz_PB_W(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Fz_PB_W_pool_[body_node_index];
  }

  SpatialForce<T>& get_mutable_Fz_PB_W(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Fz_PB_W_pool_[body_node_index];
  }

  const Matrix6X<T>& get_G_W(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return G_W_pool_[body_node_index];
  }

  Matrix6X<T>& get_mutable_G_W(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return G_W_pool_[body_node_index];
  }

  const SpatialAcceleration<T>& get_Aa_B_W(BodyNodeIndex body_node_index)
  const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Aa_B_W_pool_[body_node_index];
  }

  SpatialAcceleration<T>& get_mutable_Aa_B_W(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Aa_B_W_pool_[body_node_index];
  }

  const VectorX<T>& get_nu_W(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return nu_W_pool_[body_node_index];
  }

  VectorX<T>& get_mutable_nu_W(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return nu_W_pool_[body_node_index];
  }

 private:
  typedef std::vector<Matrix6X<T>> Matrix6X_PoolType;
  typedef std::vector<ArticulatedBodyInertia<T>> ArticulatedInertia_PoolType;
  typedef std::vector<SpatialForce<T>> SpatialForce_PoolType;
  typedef std::vector<SpatialAcceleration<T>> SpatialAcceleration_PoolType;
  typedef std::vector<VectorX<T>> VectorX_PoolType;

  void Allocate() {
    P_PB_W_pool_.resize(static_cast<unsigned long>(num_nodes_));
    Fz_PB_W_pool_.resize(static_cast<unsigned long>(num_nodes_));
    Az_PB_W_pool_.resize(static_cast<unsigned long>(num_nodes_));
    G_W_pool_.resize(static_cast<unsigned long>(num_nodes_));
    Aa_B_W_pool_.resize(static_cast<unsigned long>(num_nodes_));
    nu_W_pool_.resize(static_cast<unsigned long>(num_nodes_));
  }

  int num_nodes_{0};

  ArticulatedInertia_PoolType P_PB_W_pool_{};
  SpatialForce_PoolType Fz_PB_W_pool_{};
  SpatialAcceleration_PoolType Az_PB_W_pool_{};
  Matrix6X_PoolType G_W_pool_{};
  SpatialAcceleration_PoolType Aa_B_W_pool_{};
  VectorX_PoolType nu_W_pool_{};
};

}  // namespace multibody
}  // namespace drake