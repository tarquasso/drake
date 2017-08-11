#include "drake/examples/DoublePendulum/gen/double_pendulum_output_vector.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace examples {
namespace double_pendulum {

const int DoublePendulumOutputVectorIndices::kNumCoordinates;
const int DoublePendulumOutputVectorIndices::kTheta1;
const int DoublePendulumOutputVectorIndices::kTheta2;

const std::vector<std::string>&
DoublePendulumOutputVectorIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "theta1", "theta2",
      });
  return coordinates.access();
}

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
