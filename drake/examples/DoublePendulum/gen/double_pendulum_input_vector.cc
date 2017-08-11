#include "drake/examples/DoublePendulum/gen/double_pendulum_input_vector.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace examples {
namespace double_pendulum {

const int DoublePendulumInputVectorIndices::kNumCoordinates;
const int DoublePendulumInputVectorIndices::kTau1;
const int DoublePendulumInputVectorIndices::kTau2;

const std::vector<std::string>&
DoublePendulumInputVectorIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "tau1", "tau2",
      });
  return coordinates.access();
}

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
