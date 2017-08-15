#include "drake/examples/multi_pendulum/gen/multi_pendulum_input_vector.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace examples {
namespace multi_pendulum {

const int MultiPendulumInputVectorIndices::kNumCoordinates;
const int MultiPendulumInputVectorIndices::kTau1;
const int MultiPendulumInputVectorIndices::kTau2;

const std::vector<std::string>&
MultiPendulumInputVectorIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "tau1", "tau2",
      });
  return coordinates.access();
}

}  // namespace multi_pendulum
}  // namespace examples
}  // namespace drake
