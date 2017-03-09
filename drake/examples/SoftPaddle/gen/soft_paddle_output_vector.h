#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"

namespace drake{
namespace examples{
namespace soft_paddle{

/// Describes the row indices of a Soft_paddleOutputVector.
struct Soft_paddleOutputVectorIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
static const int kTheta1 = 0;
static const int kTheta2 = 1;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class Soft_paddleOutputVector : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef Soft_paddleOutputVectorIndices K;

  /// Default constructor.  Sets all rows to zero.
  Soft_paddleOutputVector() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  Soft_paddleOutputVector<T>* DoClone() const override {
    auto result = new Soft_paddleOutputVector;
    result->set_value(this->get_value());
    return result;
  }

  /// @name Getters and Setters
  //@{
    /// theta1
    const T& theta1() const { return this->GetAtIndex(K::kTheta1); }
    void set_theta1(const T& theta1) {
      this->SetAtIndex(K::kTheta1, theta1);
    }
    /// theta2
    const T& theta2() const { return this->GetAtIndex(K::kTheta2); }
    void set_theta2(const T& theta2) {
      this->SetAtIndex(K::kTheta2, theta2);
    }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(theta1());
    result = result && !isnan(theta2());
    return result;
  }

};

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
