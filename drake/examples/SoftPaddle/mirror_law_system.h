#pragma once

#include <cstdint>
#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/examples/SoftPaddle/soft_paddle_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace examples {
namespace soft_paddle {

template <typename T>
class PaddleMirrorLawSystem : public systems::LeafSystem<T> {
 public:
  /// Constructs a system with a vector output that is constant and equals the
  /// supplied @p source_value at all times.
  /// @param source_value the constant value of the output so that
  /// `y = source_value` at all times.
  PaddleMirrorLawSystem(const T& phi0, const T& amplitude);

  bool has_any_direct_feedthrough() const override { return false; }

  /// Returns the output port to the constant source.
  const systems::OutputPortDescriptor<T>& get_paddle_angle_port() const;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

 private:
  T phi0_{0.0};
  T amplitude_{0.1};
};

template <typename T>
class SoftPaddleWithMirrorControl : public systems::Diagram<T> {
 public:
  SoftPaddleWithMirrorControl(const T& phi0, const T& amplitude);

  /// Returns the output port for visualization with a BotVisualizer.
  const systems::OutputPortDescriptor<T>& get_paddle_state_port() const;

  const systems::OutputPortDescriptor<T>& get_paddle_angle_port() const;

  const systems::OutputPortDescriptor<T>& get_elements_port() const;

  /// Returns a reference to a RigidBodyTree model that can be used for
  /// visualization of the paddle system with a BotVisualizer.
  const RigidBodyTree<double>& get_rigid_body_tree_model() const {
    return paddle_->get_rigid_body_tree_model();
  }

  void set_initial_conditions(systems::Context<T>* context,
                              const T& x0, const T& z0) const;

  SoftPaddleStateVector<T>* GetMutablePlantStateVector(
      systems::Context<T>* context) const;

  const SoftPaddlePlant<T>& get_soft_paddle_plant() const { return *paddle_; }

 private:
  T phi0_{0.0};
  T amplitude_{0.0};
  SoftPaddlePlant<T>* paddle_;
};

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
