// Copyright 2016 Robot Locomotion Group @ CSAIL. All rights reserved.
#pragma once

// This file is generated by a script.  Do not edit!
// See drake/examples/Cars/lcm_vector_gen.py.

#include <stdexcept>
#include <string>
#include <Eigen/Core>

#include "lcmtypes/drake/lcmt_simple_car_state_t.hpp"

namespace drake {

/// Describes the row indices of a SimpleCarState.
struct SimpleCarStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordiates = 4;

  // The index of each individual coordinate.
  static const int kX = 0;
  static const int kY = 1;
  static const int kHeading = 2;
  static const int kVelocity = 3;
};

/// Models the Drake::LCMVector concept.
template <typename ScalarType = double>
class SimpleCarState {
 public:
  // An abbreviation for our row index constants.
  typedef SimpleCarStateIndices K;

  /// @name Getters and Setters
  //@{
  const ScalarType& x() const { return value_(K::kX); }
  void set_x(const ScalarType& x) { value_(K::kX) = x; }
  const ScalarType& y() const { return value_(K::kY); }
  void set_y(const ScalarType& y) { value_(K::kY) = y; }
  const ScalarType& heading() const { return value_(K::kHeading); }
  void set_heading(const ScalarType& heading) { value_(K::kHeading) = heading; }
  const ScalarType& velocity() const { return value_(K::kVelocity); }
  void set_velocity(const ScalarType& velocity) {
    value_(K::kVelocity) = velocity;
  }
  //@}

  /// @name Implement the Drake::Vector concept.
  //@{

  // Even though in practice we have a fixed size, we declare
  // ourselves dynamically sized for compatibility with the
  // system/framework/vector_interface.h API, and so that we
  // can avoid the alignment issues that come into play with
  // a fixed-size Eigen::Matrix member field.
  static const int RowsAtCompileTime = Eigen::Dynamic;
  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;
  size_t size() const { return K::kNumCoordiates; }

  /// Default constructor.  Sets all rows to zero.
  SimpleCarState()
      : value_(Eigen::Matrix<ScalarType, K::kNumCoordiates, 1>::Zero()) {}

  /// Implicit Eigen::Matrix conversion.
  template <typename Derived>
  // NOLINTNEXTLINE(runtime/explicit) per Drake::Vector.
  SimpleCarState(const Eigen::MatrixBase<Derived>& value)
      : value_(value.segment(0, K::kNumCoordiates)) {}

  /// Eigen::Matrix assignment.
  template <typename Derived>
  SimpleCarState& operator=(const Eigen::MatrixBase<Derived>& value) {
    value_ = value.segment(0, K::kNumCoordiates);
    return *this;
  }

  /// Magic conversion specialization back to Eigen.
  friend EigenType toEigen(const SimpleCarState<ScalarType>& vec) {
    return vec.value_;
  }

  /// Magic pretty names for our coordinates.  (This is an optional
  /// part of the Drake::Vector concept, but seems worthwhile.)
  friend std::string getCoordinateName(const SimpleCarState<ScalarType>& vec,
                                       unsigned int index) {
    switch (index) {
      case K::kX:
        return "x";
      case K::kY:
        return "y";
      case K::kHeading:
        return "heading";
      case K::kVelocity:
        return "velocity";
    }
    throw std::domain_error("unknown coordinate index");
  }

  //@}

  /// @name Implement the LCMVector concept
  //@{
  typedef drake::lcmt_simple_car_state_t LCMMessageType;
  static std::string channel() { return "SIMPLE_CAR_STATE"; }
  //@}

 private:
  EigenType value_;
};

template <typename ScalarType>
bool encode(const double& t, const SimpleCarState<ScalarType>& wrap,
            // NOLINTNEXTLINE(runtime/references)
            drake::lcmt_simple_car_state_t& msg) {
  msg.timestamp = static_cast<int64_t>(t * 1000);
  msg.x = wrap.x();
  msg.y = wrap.y();
  msg.heading = wrap.heading();
  msg.velocity = wrap.velocity();
  return true;
}

template <typename ScalarType>
bool decode(const drake::lcmt_simple_car_state_t& msg,
            // NOLINTNEXTLINE(runtime/references)
            double& t,
            // NOLINTNEXTLINE(runtime/references)
            SimpleCarState<ScalarType>& wrap) {
  t = static_cast<double>(msg.timestamp) / 1000.0;
  wrap.set_x(msg.x);
  wrap.set_y(msg.y);
  wrap.set_heading(msg.heading);
  wrap.set_velocity(msg.velocity);
  return true;
}

}  // namespace drake
