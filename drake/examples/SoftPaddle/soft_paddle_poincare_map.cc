#include "drake/examples/SoftPaddle/soft_paddle_poincare_map-inl.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#include <iomanip> //allowing to set precision of the PRINT_VAR outputs to terminal
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace examples {
namespace soft_paddle {

template class SoftPaddlePoincareMap<double>;
// Eigen's tan fails at runtime if using AutoDiffXd.
// As a quick fix I am using a fixed size AutoDiffScalar.
//template class SoftPaddlePoincareMap<AutoDiffXd>;

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake