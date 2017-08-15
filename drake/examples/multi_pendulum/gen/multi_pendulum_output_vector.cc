#include "drake/examples/multi_pendulum/gen/multi_pendulum_output_vector.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace examples {
namespace multi_pendulum {

const int MultiPendulumOutputVectorIndices::kNumCoordinates;
const int MultiPendulumOutputVectorIndices::kTheta1;
const int MultiPendulumOutputVectorIndices::kTheta2;

const std::vector<std::string>&
MultiPendulumOutputVectorIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "theta1", "theta2",
      });
  return coordinates.access();
}

}  // namespace multi_pendulum
}  // namespace examples
}  // namespace drake
