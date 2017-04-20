#include "drake/examples/SoftPaddle/soft_paddle_plant-inl.h"

namespace drake {
namespace examples {
namespace soft_paddle {

template class SoftPaddlePlant<double>;
// Eigen's tan fails at runtime if using AutoDiffXd.
// As a quick fix I am using a fixed size AutoDiffScalar.
template class SoftPaddlePlant<AutoDiffXd>;

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
