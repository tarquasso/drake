#include "drake/examples/DoublePendulum/gen/double_pendulum_state_vector.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace examples {
namespace double_pendulum {

const int DoublePendulumStateVectorIndices::kNumCoordinates;
const int DoublePendulumStateVectorIndices::kTheta1;
const int DoublePendulumStateVectorIndices::kTheta2;
const int DoublePendulumStateVectorIndices::kTheta1dot;
const int DoublePendulumStateVectorIndices::kTheta2dot;

const std::vector<std::string>&
DoublePendulumStateVectorIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "theta1", "theta2", "theta1dot", "theta2dot",
      });
  return coordinates.access();
}

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
