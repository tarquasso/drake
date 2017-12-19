#include "multibody/multibody_tree/articulated_kinematics_cache.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class ArticulatedKinematicsCache<double>;
template class ArticulatedKinematicsCache<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake