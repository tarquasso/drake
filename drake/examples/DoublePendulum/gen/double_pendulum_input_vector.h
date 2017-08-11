#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/never_destroyed.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace examples {
namespace double_pendulum {

/// Describes the row indices of a DoublePendulumInputVector.
struct DoublePendulumInputVectorIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kTau1 = 0;
  static const int kTau2 = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words,
  /// `DoublePendulumInputVectorIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class DoublePendulumInputVector : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef DoublePendulumInputVectorIndices K;

  /// Default constructor.  Sets all rows to zero.
  DoublePendulumInputVector() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  DoublePendulumInputVector<T>* DoClone() const override {
    return new DoublePendulumInputVector;
  }

  /// @name Getters and Setters
  //@{
  /// tau1
  const T& tau1() const { return this->GetAtIndex(K::kTau1); }
  void set_tau1(const T& tau1) { this->SetAtIndex(K::kTau1, tau1); }
  /// tau2
  const T& tau2() const { return this->GetAtIndex(K::kTau2); }
  void set_tau2(const T& tau2) { this->SetAtIndex(K::kTau2, tau2); }
  //@}

  /// See DoublePendulumInputVectorIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return DoublePendulumInputVectorIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(tau1());
    result = result && !isnan(tau2());
    return result;
  }
};

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
