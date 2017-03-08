#pragma once

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace examples {
namespace soft_paddle {

/// Describes the row indices of a PendulumStateVector.
struct SoftPaddleStateVectorIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kX = 0;
  static const int kZ = 1;

  static const int kXdot = 2;
  static const int kZdot = 3;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class SoftPaddleStateVector : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef SoftPaddleStateVectorIndices K;

  /// Default constructor.  Sets all rows to zero.
  SoftPaddleStateVector() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  // x coordinate
  const T& x() const { return this->GetAtIndex(K::kX); }
  void set_x(const T& x) { this->SetAtIndex(K::kX, x); }
  // z coordinate
  const T& z() const { return this->GetAtIndex(K::kZ); }
  void set_z(const T& z) { this->SetAtIndex(K::kZ, z); }

  // xdot
  const T& xdot() const { return this->GetAtIndex(K::kXdot); }
  void set_xdot(const T& x) { this->SetAtIndex(K::kXdot, x); }
  // zdot
  const T& zdot() const { return this->GetAtIndex(K::kZdot); }
  void set_zdot(const T& z) { this->SetAtIndex(K::kZdot, z); }
  //@}
};

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
