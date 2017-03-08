#include "drake/examples/SoftPaddle/soft_paddle_state_to_bot_visualizer.h"

#include "drake/examples/SoftPaddle/soft_paddle_state_vector.h"
#include "drake/multibody/rigid_body_tree.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace examples {
namespace soft_paddle {

using systems::Context;
using systems::kVectorValued;
using systems::System;
using systems::SystemOutput;

namespace {
// q = [x, z], xc = [q, qdot]
constexpr int kStateSize = SoftPaddleStateVectorIndices::kNumCoordinates;

// 1 revolute joint for the paddle = 2 states.
// 1 quaternion joint for the disk = 13 (= 7 + 6) states.
//
constexpr int kVisualizerStateSize =
    15 + kNumPaddleElements * 13;
}

template <typename T>
SoftPaddleStateToBotVisualizer<T>::SoftPaddleStateToBotVisualizer(
    const SoftPaddlePlant<T>& plant) :
    rbt_model_(plant.get_rigid_body_tree_model()),
    x0_(plant.get_default_x0()),
    z0_(plant.get_default_z0()){
  // Input for the SoftPaddlePlant state.
  this->DeclareInputPort(systems::kVectorValued, kStateSize);

  // Input for the paddle angle.
  this->DeclareInputPort(kVectorValued, 1);
  // Input for the paddle as a collection of small rigid elements.
  // 3D position only.
  this->DeclareInputPort(
      systems::kVectorValued, 4 * kNumPaddleElements);

  // Output for the BotVisualizer.
  this->DeclareOutputPort(kVectorValued, kVisualizerStateSize);
}

template <typename T>
const systems::InputPortDescriptor<T>&
SoftPaddleStateToBotVisualizer<T>::get_paddle_angle_port() const {
  return System<T>::get_input_port(1);
}

template <typename T>
const systems::InputPortDescriptor<T>&
SoftPaddleStateToBotVisualizer<T>::get_paddle_state_port() const {
  return System<T>::get_input_port(0);
}

template <typename T>
const systems::InputPortDescriptor<T>&
SoftPaddleStateToBotVisualizer<T>::get_elements_port() const {
  return System<T>::get_input_port(2);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
SoftPaddleStateToBotVisualizer<T>::get_bot_visualizer_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
void SoftPaddleStateToBotVisualizer<T>::DoCalcOutput(const Context<T>& context,
                                         SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // Input from the paddle plant.
  const SoftPaddleStateVector<T>* state =
      dynamic_cast<const SoftPaddleStateVector<T>*>(
          this->EvalVectorInput(context, 0));

  // Input angle.
  T phi = this->EvalVectorInput(context, 1)->GetAtIndex(0);

  output->GetMutableVectorData(0)->SetFromVector(
      VectorX<T>::Zero(kVisualizerStateSize));

  // Revolute joint.
  output->GetMutableVectorData(0)->SetAtIndex(0, phi);
  // Quaternion for the disk.
  output->GetMutableVectorData(0)->SetAtIndex(1, state->x() - x0_);
  output->GetMutableVectorData(0)->SetAtIndex(2, 0.0);
  output->GetMutableVectorData(0)->SetAtIndex(3, state->z() - z0_);
  output->GetMutableVectorData(0)->SetAtIndex(4, 1.0);
  output->GetMutableVectorData(0)->SetAtIndex(5, 0.0);
  output->GetMutableVectorData(0)->SetAtIndex(6, 0.0);
  output->GetMutableVectorData(0)->SetAtIndex(7, 0.0);

  // Flexible elements.
  // Input angle.
  auto xe = this->EvalVectorInput(context, 2);
  auto xv = output->GetMutableVectorData(0);
  for (int i = 0; i < kNumPaddleElements; ++i) {
    double xe0 = i * (0.7 / (kNumPaddleElements - 1));
    //PRINT_VAR(xe->GetAtIndex(3 * i + 0));
    //PRINT_VAR(xe->GetAtIndex(3 * i + 1));
    //PRINT_VAR(xe->GetAtIndex(3 * i + 2));
    xv->SetAtIndex(8 + 7 * i + 0, xe->GetAtIndex(4 * i + 0) - xe0);
    xv->SetAtIndex(8 + 7 * i + 1, xe->GetAtIndex(4 * i + 1));
    xv->SetAtIndex(8 + 7 * i + 2, xe->GetAtIndex(4 * i + 2));
    Eigen::Quaterniond q =
        Eigen::AngleAxisd(xe->GetAtIndex(4 * i + 3), Eigen::Vector3d::UnitY())
        *Eigen::AngleAxisd::Identity();
    //PRINT_VAR(xe->GetAtIndex(4 * i + 3));
    xv->SetAtIndex(8 + 7 * i + 3, q.w());
    xv->SetAtIndex(8 + 7 * i + 4, q.x());
    xv->SetAtIndex(8 + 7 * i + 5, q.y());
    xv->SetAtIndex(8 + 7 * i + 6, q.z());
  }
}

// Explicitly instantiates on the most common scalar types.
template class SoftPaddleStateToBotVisualizer<double>;

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
