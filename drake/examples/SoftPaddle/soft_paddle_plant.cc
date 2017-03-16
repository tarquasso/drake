#include "drake/examples/SoftPaddle/soft_paddle_plant.h"

#include <memory>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/joints/revolute_joint.h"

namespace drake {
namespace examples {
namespace soft_paddle {

using std::make_unique;
using std::unique_ptr;

using DrakeShapes::Box;
using DrakeShapes::Cylinder;
using DrakeShapes::VisualElement;

using Eigen::Isometry3d;
using Eigen::Rotation2D;
using Eigen::Vector3d;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace {
// q = [x, z], xc = [q, qdot]
constexpr int kStateSize = 4;

// 1 revolute joint for the paddle = 2 states.
// 1 quaternion joint for the disk = 13 (= 7 + 6) states.
//constexpr int kVisualizerStateSize = 15 + kNumPaddleElements * 3;
}

template <typename T>
SoftPaddlePlant<T>::SoftPaddlePlant() {
  // Input port for the paddle angle.
  this->DeclareInputPort(
      systems::kVectorValued, 1);

  // Outputs the state.
  this->DeclareOutputPort(
      systems::kVectorValued, kStateSize);

  // Output for the paddle as a collection of small rigid elements.
  // 3D position only.
  this->DeclareOutputPort(
      systems::kVectorValued, 4 * kNumPaddleElements);

  element_positions_ = VectorX<T>::Zero(3 * kNumPaddleElements);
  element_angles_ = VectorX<T>::Zero(kNumPaddleElements);

  for(int i = 0; i < kNumPaddleElements; ++i) {
    // Positions in paddle frame.
    T xe_p = i * ell_/kNumPaddleElements;
    element_positions_.segment(3 * i, 3) = Vector3<T>(xe_p, 0.0, 0.0);
  }

  CreateRBTModel();
}

template <typename T>
SoftPaddlePlant<T>::~SoftPaddlePlant() {}

template <typename T>
const systems::InputPortDescriptor<T>&
SoftPaddlePlant<T>::get_tau_port() const {
  return this->get_input_port(0);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
SoftPaddlePlant<T>::get_output_port() const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
SoftPaddlePlant<T>::get_elements_port() const {
  return systems::System<T>::get_output_port(1);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
SoftPaddlePlant<T>::get_visualizer_output_port() const {
  return systems::System<T>::get_output_port(1);
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
SoftPaddlePlant<T>::AllocateOutputVector(
    const systems::OutputPortDescriptor<T>& descriptor) const {
  if(descriptor.get_index() == 0) {
    return std::make_unique<SoftPaddleStateVector<T>>();
  } else {
    return std::make_unique<systems::BasicVector<T>>(4 * kNumPaddleElements);
  }
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
SoftPaddlePlant<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<SoftPaddleStateVector<T>>(),
      2 /* num_q */, 2 /* num_v */, 0 /* num_z */);
  static_assert(kStateSize == 2 + 2, "State size has changed");
}

template <typename T>
void SoftPaddlePlant<T>::DoCalcOutput(const systems::Context<T>& context,
                                   systems::SystemOutput<T>* output) const {
  // Set output 0: state.
  get_mutable_output(output)->set_value(get_state(context).get_value());

  // Elements positions.
  VectorX<T> elements_state(4 * kNumPaddleElements);
  //elements_state << element_positions_, element_angles_;
  for (int i = 0;i < kNumPaddleElements; ++i) {
    elements_state.segment(4 * i, 3) = element_positions_.segment(3 * i, 3);
    elements_state[4 * i + 3] = element_angles_[i];
  }
  output->GetMutableVectorData(1)->SetFromVector(elements_state);
}

// Compute the actual physics.
template <typename T>
void SoftPaddlePlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  const SoftPaddleStateVector<T>& state = get_state(context);
  SoftPaddleStateVector<T>* derivative_vector = get_mutable_state(derivatives);

  derivative_vector->set_x(state.xdot());
  derivative_vector->set_z(state.zdot());

  // =========================================================
  // Only consider disk's dynamics. And assume phi = 0.
  // =========================================================

  // Disk coordinates in world's frame.
  const T x = state.x();
  const T z = state.z();
  const T xdot = state.xdot();
  const T zdot = state.zdot();
  const T phi = get_paddle_angle(context);

  // Transform into paddle's frame (assuming now phi = 0)
  Matrix2<T> R_wp = Rotation2D<T>(-phi).matrix();
  Vector2<T> xvec_p = R_wp * Vector2<T>(x, z);
  T x_p = xvec_p.x();
  T z_p = xvec_p.y();

  Vector2<T> xdotvec_p = R_wp * Vector2<T>(xdot, zdot);
  // T xdot_p = xdotvec_p.x();
  T zdot_p = xdotvec_p.y();

  T delta = z_p - Rd_;  // Penetration distance into undeformed paddle.
  // Compute elastic interaction force.
  T Fx = 0.;  // Horizontal force in the world's frame.
  T Fz = 0.;  // Vertical force in the world's frame.

  //PRINT_VAR(x);
  //PRINT_VAR(z);
  //PRINT_VAR(delta);

  T theta1 = 0.0;
  T theta2 = 0.0;

  if(delta < 0 && 0.001 < x_p && x_p < ell_ - 0.001) {

    // Angle between the rubber band and the horizontal on the left side.
    theta1 = - delta / x_p;  // Always positive.

    // Angle between the rubber band and the horizontal on the right side.
    theta2 = - delta / (ell_ - x_p);  // Always positive.

    // Angle between the two straight sections of the rubber band at each side
    // of the disk.
    T theta = theta1 + theta2;

    // Angle between the contact force and the vertical.
    // x > ell_ / 2 ==> beta > 0. Force points to the left.
    // x < ell_ / 2 ==> beta < 0. Force points to the right.
    T beta = (theta2 - theta1) / 2.0;

    // Forces acting on the disk in the frame of the paddle.
    T Fz_p = T0_ * theta;
    T Fx_p = - Fz_p * beta;

    // Damping force (find a better model).
    Fz_p += -damping_coefficient_ * zdot_p;

    // Force in worlds's frame (assuming now phi = 0)
    Vector2<T> Fvec_w = R_wp.transpose() * Vector2<T>(Fx_p, Fz_p);
    Fx = Fvec_w.x();
    Fz = Fvec_w.y();
  }

  for(int i = 0; i < kNumPaddleElements; ++i) {
    // Positions in paddle frame.
    T xe_p = i * (ell_/(kNumPaddleElements-1));
    T ze_p = 0.0;
    // Linear
    /*
    if(xe_p < x_p) {
      ze_p =  - xe_p * tan(theta1);
      element_angles_[i] = -(phi - theta1);
    } else {
      ze_p =  - (ell_ - xe_p) * tan(theta2);
      element_angles_[i] = -(phi + theta2);
    }*/

      // Large deformation
      // See related geometry problem here:
      // http://math.stackexchange.com/questions/1064410/what-is-the-radius-of-a-circle-tangent-to-two-lines-with-a-known-angle-between-t
      // Esentially this is the same problem but rotated an angle beta below
      // the horizontal, where beta = atan(zd_p, xd_p) with (zd_p, xd_p) the
      // coordinates of the disk in the paddle frame.
      // Once beta and alpha/2 are found in the paddle frame, the solution is
      // rotated by the paddle angle phi to the world frame.
      Vector2<T> xe_w;
      T x1 = xe_p; // Coordiantes of contact
      T x2 = xe_p; // Coordiantes of contact
      theta1 = 0.0;
      theta2 = 0.0;
      if( delta < 0 ) {
          // Left
          {
              Vector2<T> x_center(x_p, z_p);
              T d = x_center.norm();
              T alpha_2 = asin(Rd_ / d); // alpha / 2
              T beta = atan(-z_p /
                            x_p);  // angle between x_center and the horizontal.
              theta1 = alpha_2 +
                       beta; // angle between tangent line and horizontal.
              T xc_dist = sqrt(d * d - Rd_ * Rd_); // Distance to contact.
              Vector2<T> xc = xc_dist * Vector2<T>(cos(theta1), -sin(theta1));
              x1 = xc[0];
          }
          // Right
          {
              Vector2<T> x_center(-(ell_-x_p), z_p);
              T d = x_center.norm();
              T alpha_2 = asin(Rd_ / d); // alpha / 2
              // angle between x_center and the horizontal.
              T beta = atan(-z_p / (ell_-x_p));
              theta2 = alpha_2 +
                       beta; // angle between tangent line and horizontal.
              T xc_dist = sqrt(d * d - Rd_ * Rd_); // Distance to contact.
              Vector2<T> xc = xc_dist * Vector2<T>(-cos(theta2), -sin(theta2));
              x2 = ell_ + xc[0];
          }

          if (xe_p < x1) { // Left
              ze_p = -xe_p * tan(theta1);
              element_angles_[i] = -(phi - theta1);
          } else if (xe_p > x2) { // Right
              ze_p = -(ell_ - xe_p) * tan(theta2);
              element_angles_[i] = -(phi + theta2);
          } else { // Center
              ze_p = z_p - sqrt(Rd_*Rd_ - (xe_p-x_p)*(xe_p-x_p));
              element_angles_[i] = atan((xe_p-x_p)/(ze_p-z_p))-phi;
          }
      } else {
          element_angles_[i] = -phi;
      }

      // Rotate to world's frame.
      xe_w = R_wp.transpose() * Vector2<T>(xe_p, ze_p);


      element_positions_.segment(3 * i, 3) =
              Vector3<T>(xe_w[0], 0.0, xe_w[1]);

    //PRINT_VAR(xe_w.transpose());
  }  // loop on elements.

  //PRINT_VAR(Fx);
 // PRINT_VAR(Fz);

  // Disk's acceleration.
  derivative_vector->set_xdot(Fx);
  derivative_vector->set_zdot((Fz - md_ * g_));
}

// SoftPaddlePlant has no constructor arguments, so there's no work to do here.
//template <typename T>
//SoftPaddlePlant<AutoDiffXd>* SoftPaddlePlant<T>::DoToAutoDiffXd() const {
//  return new SoftPaddlePlant<AutoDiffXd>();
//}

template <typename T>
void SoftPaddlePlant<T>::CreateRBTModel() {
  rbt_model_ = std::make_unique<RigidBodyTree<double>>();

  Eigen::Vector4d red(0.9, 0.1, 0.0, 1.0);
  Eigen::Vector4d green(0.3, 0.6, 0.4, 1.0);
  Eigen::Vector4d blue(0.0, 0.0, 1.0, 1.0);
  Eigen::Vector4d grey(0.5, 0.5, 0.5, 1.0);
  Eigen::Vector4d white(1.0, 1.0, 1.0, 1.0);
  Eigen::Vector4d black(0.0, 0.0, 0.0, 1.0);

  // Adds world's visuals.
  {
    RigidBody<double>& body = rbt_model_->world();

    // Table.
    Isometry3d pose = Isometry3d::Identity();
    pose.translation() = Vector3d(0.5 * ell_, 0.0625, 0.25 * ell_);
    DrakeShapes::VisualElement table_visual(
        Box(Vector3d(1.4 * ell_, 0.025, 1.4 * ell_)), pose, white);
    body.AddVisualElement(table_visual);

    // Reference horizontal line.
    pose = Isometry3d::Identity();
    pose.translation() = Vector3d(0.5 * ell_, 0.0625, 0.0);
    DrakeShapes::VisualElement horizontal_visual(
        Box(Vector3d(1.4 * ell_ + 0.002, 0.025 + 0.002, 0.005)), pose, black);
    body.AddVisualElement(horizontal_visual);
  }

  // Adds a RigidBody to model the paddle (only the rigid part).
  {
    RigidBody<double>* body =
        rbt_model_->add_rigid_body(make_unique<RigidBody<double>>());
    body->set_name("body");
    // Sets body to have a non-zero spatial inertia. Otherwise the body gets
    // welded by a fixed joint to the world by RigidBodyTree::compile().
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    // Paddle visual.
    Isometry3d pose = Isometry3d::Identity();
    pose.translation() = Vector3d(0.35, 0.1/2, 0.0);
    DrakeShapes::VisualElement visual_element(
        Box(Vector3d(ell_, 0.015, 0.05)), pose, red);
    body->AddVisualElement(visual_element);

    // Shaft visual.
    pose = Isometry3d::Identity();
    pose.linear() =
        Eigen::AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();
    DrakeShapes::VisualElement shaft_visual(
        Cylinder(0.03, 0.15), pose, grey);
    body->AddVisualElement(shaft_visual);

    // Paddle angle.
    body->add_joint(&rbt_model_->world(),
                      make_unique<RevoluteJoint>(
                          "phi", Isometry3d::Identity(), -Vector3d::UnitY()));
  }

  // Adds a RigidBody to model the disk.
  {
    RigidBody<double>* body =
        rbt_model_->add_rigid_body(make_unique<RigidBody<double>>());
    body->set_name("disk");
    // Sets body to have a non-zero spatial inertia. Otherwise the body gets
    // welded by a fixed joint to the world by RigidBodyTree::compile().
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    Isometry3d pose = Isometry3d::Identity();
    pose.translation() = Vector3d(x0_, 0.0, z0_);
    pose.linear() =
        Eigen::AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();
    DrakeShapes::VisualElement visual_element(
        Cylinder(Rd_, 0.025), pose, green);
    body->AddVisualElement(visual_element);

    body->add_joint(&rbt_model_->world(),
                    make_unique<QuaternionFloatingJoint>(
                        "phi", Isometry3d::Identity()));
  }

  // Adds a set of small boxes to model the deformable paddle (only the rigid
  // part).
  {
    for(int i_element = 0; i_element < kNumPaddleElements; ++i_element) {
      RigidBody<double>* body =
          rbt_model_->add_rigid_body(make_unique<RigidBody<double>>());
      std::string element_name = "paddle_element_"+std::to_string(i_element);
      body->set_name(element_name);
      double dell = ell_ / (kNumPaddleElements - 1);
      double xe = i_element * dell;
      //PRINT_VAR(xe);
      //body->set_center_of_mass(Vector3d(xe, 0.0, 0.0));
      // Sets body to have a non-zero spatial inertia. Otherwise the body gets
      // welded by a fixed joint to the world by RigidBodyTree::compile().
      body->set_mass(1.0);
      body->set_spatial_inertia(Matrix6<double>::Identity());

      Isometry3d pose = Isometry3d::Identity();
      //pose.translation() = Vector3d(xe, 0.0, 0.0);
      //body->get_center_of_mass();
      //pose.linear() =
      //    Eigen::AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();
      DrakeShapes::VisualElement visual_element(
          Box(Vector3d(dell*1.2, 0.1, 0.005)), pose, blue);
          //Cylinder(Rd_/5.0, 0.1), pose, blue);
      body->AddVisualElement(visual_element);

      pose.translation() = Vector3d(xe, 0.0, 0.0);
      body->add_joint(&rbt_model_->world(),
                      make_unique<QuaternionFloatingJoint>(
                          element_name, pose));

      //body->add_joint(&rbt_model_->world(),
      //                make_unique<FixedJoint>(
      //                    element_name, Isometry3d::Identity()));
    }
  }

  rbt_model_->compile();
}

template <typename T>
void SoftPaddlePlant<T>::set_initial_conditions(MyContext* context) const {
  auto state =
      dynamic_cast<SoftPaddleStateVector<T>*>(
          context->get_mutable_continuous_state_vector());
  state->SetFromVector(VectorX<T>::Zero(kStateSize));
  state->set_x(x0_);
  state->set_z(z0_);
}

template class SoftPaddlePlant<double>;

// Eigen's tan fails at runtime if using AutoDiffXd.
// As a quick fix I am using a fixed size AutoDiffScalar.
template class SoftPaddlePlant<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class SoftPaddlePlant<Eigen::AutoDiffScalar<Eigen::Vector4d>>;
template class SoftPaddlePlant<AutoDiffXd>;

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
