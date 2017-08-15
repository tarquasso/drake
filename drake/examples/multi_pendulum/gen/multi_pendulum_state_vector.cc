#include "drake/examples/multi_pendulum/gen/multi_pendulum_state_vector.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace examples {
namespace multi_pendulum {

const int MultiPendulumStateVectorIndices::kNumCoordinates;
const int MultiPendulumStateVectorIndices::kTheta1;
const int MultiPendulumStateVectorIndices::kTheta2;
const int MultiPendulumStateVectorIndices::kTheta1dot;
const int MultiPendulumStateVectorIndices::kTheta2dot;

const std::vector<std::string>&
MultiPendulumStateVectorIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "theta1", "theta2", "theta1dot", "theta2dot",
      });
  return coordinates.access();
}

}  // namespace multi_pendulum
}  // namespace examples
}  // namespace drake
