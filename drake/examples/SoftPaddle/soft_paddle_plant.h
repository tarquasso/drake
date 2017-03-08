#pragma once

#include <memory>

#include "drake/examples/SoftPaddle/soft_paddle_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace soft_paddle {

const int kNumPaddleElements{150};

/// A model of a simple pendulum
/// @f[ ml^2 \ddot\theta + b\dot\theta + mgl\sin\theta = u @f]
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
template <typename T>
class SoftPaddlePlant : public systems::LeafSystem<T> {
 public:
  SoftPaddlePlant();
  ~SoftPaddlePlant() override;

  using MyContext = systems::Context<T>;
  using MyContinuousState = systems::ContinuousState<T>;
  using MyOutput = systems::SystemOutput<T>;

  /// The input force to this system is not direct feedthrough.
  bool has_any_direct_feedthrough() const override { return false; }

  /// Returns the input port to the externally applied torque.
  const systems::InputPortDescriptor<T>& get_tau_port() const;

  /// Returns the port to output state.
  const systems::OutputPortDescriptor<T>& get_output_port() const;

  const systems::OutputPortDescriptor<T>& get_elements_port() const;

  /// Returns the port to output state.
  const systems::OutputPortDescriptor<T>& get_visualizer_output_port() const;

#if 0
  void set_theta(MyContext* context, const T& theta) const {
    get_mutable_state(context)->set_theta(theta);
  }

  void set_thetadot(MyContext* context, const T& thetadot) const {
    get_mutable_state(context)->set_thetadot(thetadot);
  }
#endif

  /// Paddle mass in Kg.
  double get_default_paddle_mass() const { return mp_; }
  /// Paddle length in meters.
  double get_default_paddle_length() const { return ell_; }
  /// Gravity in m/s^2.
  double get_default_gravity() const { return g_; }

  double get_default_x0() const { return x0_; }

  double get_default_z0() const { return z0_; }


  /// Returns a reference to a RigidBodyTree model that can be used for
  /// visualization of the paddle system with a BotVisualizer.
  const RigidBodyTree<double>& get_rigid_body_tree_model() const {
    return *rbt_model_.get();
  }

  void set_initial_conditions(MyContext* context) const;

  explicit SoftPaddlePlant(const SoftPaddlePlant& other) = delete;
  SoftPaddlePlant& operator=(const SoftPaddlePlant& other) = delete;
  explicit SoftPaddlePlant(SoftPaddlePlant&& other) = delete;
  SoftPaddlePlant& operator=(SoftPaddlePlant&& other) = delete;

 protected:
  // LeafSystem<T> override.
  std::unique_ptr<MyContinuousState>
  AllocateContinuousState() const override;

  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::OutputPortDescriptor<T>& descriptor) const override;

  // System<T> override.
  //SoftPaddlePlant<AutoDiffXd>* DoToAutoDiffXd() const override;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void DoCalcTimeDerivatives(
      const MyContext& context,
      MyContinuousState* derivatives) const override;

 private:
  T get_paddle_angle(const MyContext &context) const {
    return this->EvalVectorInput(context, 0)->GetAtIndex(0);
  }

  static const SoftPaddleStateVector<T>& get_state(
      const MyContinuousState& cstate) {
    return dynamic_cast<const SoftPaddleStateVector<T>&>(cstate.get_vector());
  }

  static SoftPaddleStateVector<T>* get_mutable_state(
      MyContinuousState* cstate) {
    return dynamic_cast<SoftPaddleStateVector<T>*>(cstate->get_mutable_vector());
  }

  static SoftPaddleStateVector<T>* get_mutable_output(MyOutput* output) {
    return dynamic_cast<SoftPaddleStateVector<T>*>(
        output->GetMutableVectorData(0));
  }

  static const SoftPaddleStateVector<T>& get_state(const MyContext& context) {
    return get_state(*context.get_continuous_state());
  }

  static SoftPaddleStateVector<T>* get_mutable_state(MyContext* context) {
    return get_mutable_state(context->get_mutable_continuous_state());
  }

  // Creates a RigidBodyTree model of the paddle. This RigidBodyTree is used
  // with a BotVisualizer system to visualize the paddle system.
  void CreateRBTModel();

  double g_{9.81};  // Acceleration of gravity.

  // Paddle parameters.
  double Ip_{0.0};  // Paddle moment of inertia. [kg * m^2]
  double ell_{0.7}; // Paddle/rubber band length. [m]
  double mp_{0.0};  // Paddle mass. [Kg]
  double T0_{10.0};  // Rubber band tension. [N]

  // Disk parameters.
  double Rd_{0.04};  // Disk's radius. [m]
  double md_{0.1};  // Disk's mass. [Kg]
  double Id_{0.5 * md_ * Rd_ * Rd_};  // Disk's moment of inertia. [Kg * m^2]

  // Initial conditions.
  double x0_{0.35}, z0_{0.4}, phi0_{5.0*M_PI/180.0};

  double damping_coefficient_{1.5};  // Rubber band damping.

  // A RigidBodyTree model of the plant for visualization.
  std::unique_ptr<RigidBodyTree<double>> rbt_model_;

  // Hacky. A vector to cache the output for the soft paddle.
  mutable drake::VectorX<T> element_positions_;
  mutable drake::VectorX<T> element_angles_;
};

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
