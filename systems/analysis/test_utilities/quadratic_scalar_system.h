#pragma once

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/state.h"

namespace drake {
namespace systems {
namespace analysis_test {

/// System where the state at (scalar) time t corresponds to the quadratic
///  equation 4t² + 4t + 3.
class QuadraticScalarSystem : public LeafSystem<double> {
 public:
  QuadraticScalarSystem() { this->DeclareContinuousState(1); }

  /// Evaluates the system at time t.
  double Evaluate(double t) const {
    return 3 + 4 * t * (t + 1);
  }

 private:
  void SetDefaultState(
      const Context<double>& context, State<double>* state) const final {
    const double t0 = 0.0;
    state->get_mutable_continuous_state().get_mutable_vector()[0] =
        Evaluate(t0);
  }

  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* deriv) const override {
    const double t = context.get_time();
    (*deriv)[0] = 8 * t + 4;
  }
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
