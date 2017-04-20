#include "drake/systems/primitives/first_order_low_pass_filter-inl.h"

namespace drake {
namespace systems {

// Explicitly instantiates on the most common scalar types.
template class FirstOrderLowPassFilter<double>;
template class FirstOrderLowPassFilter<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
