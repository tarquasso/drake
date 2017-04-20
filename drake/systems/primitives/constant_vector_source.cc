#include "drake/systems/primitives/constant_vector_source-inl.h"

namespace drake {
namespace systems {

// Explicitly instantiates on the most common scalar types.
template class ConstantVectorSource<double>;
template class ConstantVectorSource<AutoDiffXd>;
template class ConstantVectorSource<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
