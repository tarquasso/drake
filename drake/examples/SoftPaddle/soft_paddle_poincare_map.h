#pragma once

//#include "drake/systems/analysis/simulator.h"
//#include "drake/systems/framework/leaf_system.h"
#include "drake/examples/SoftPaddle/mirror_law_system.h"

namespace drake {
namespace examples {
namespace soft_paddle {

// Simple Discrete Time System
//   x[n+1] = x[n]^3
//   y = x
template <typename T>
class SoftPaddlePoincareMap  { //: public drake::systems::LeafSystem<T>
 public:
  SoftPaddlePoincareMap();

//  void DoEvalDifferenceUpdates(
//      const drake::systems::Context<T>& context,
//      drake::systems::DifferenceState<T>* updates) const override;

  // y = x
//  void EvalOutput(const drake::systems::Context<T>& context,
//                  drake::systems::SystemOutput<T>* output) const override;

  void ComputeNextSate(const T& paddle_aim, const T& stroke_strength,
                       const T& x0, const T& z0, T* xnext, T* znext) const;

 private:
  //std::unique_ptr<SoftPaddleWithMirrorControl<T>> paddle_plant_;
};

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
