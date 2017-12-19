#include "drake/multibody/multibody_tree/articulated_inertia.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class ArticulatedInertia<double>;
template class ArticulatedInertia<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
