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
namespace multi_pendulum {

/// Describes the row indices of a MultiPendulumInputVector.
struct MultiPendulumInputVectorIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kTau1 = 0;
  static const int kTau2 = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `MultiPendulumInputVectorIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class MultiPendulumInputVector : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef MultiPendulumInputVectorIndices K;

  /// Default constructor.  Sets all rows to zero.
  MultiPendulumInputVector() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  MultiPendulumInputVector<T>* DoClone() const override {
    return new MultiPendulumInputVector;
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

  /// See MultiPendulumInputVectorIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return MultiPendulumInputVectorIndices::GetCoordinateNames();
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

}  // namespace multi_pendulum
}  // namespace examples
}  // namespace drake
