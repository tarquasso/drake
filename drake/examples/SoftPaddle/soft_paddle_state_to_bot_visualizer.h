#pragma once

#include <cstdint>
#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/examples/SoftPaddle/soft_paddle_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port_value.h"

namespace drake {
namespace examples {
namespace soft_paddle {

/// A source block with a constant output port at all times.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @ingroup primitive_systems
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
template <typename T>
class SoftPaddleStateToBotVisualizer : public systems::LeafSystem<T> {
 public:
  /// Constructs a system with a vector output that is constant and equals the
  /// supplied @p source_value at all times.
  /// @param source_value the constant value of the output so that
  /// `y = source_value` at all times.
  SoftPaddleStateToBotVisualizer(const SoftPaddlePlant<T>& plant);

  bool has_any_direct_feedthrough() const override { return true; }

  const systems::OutputPortDescriptor<T>& get_bot_visualizer_port() const;

  const systems::InputPortDescriptor<T>& get_paddle_state_port() const;

  const systems::InputPortDescriptor<T>& get_elements_port() const;

  const systems::InputPortDescriptor<T>& get_paddle_angle_port() const;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

 private:
  // A RigidBodyTree model of the plant for visualization.
  const RigidBodyTree<double>& rbt_model_;
  const T x0_, z0_, ell_;
};

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
