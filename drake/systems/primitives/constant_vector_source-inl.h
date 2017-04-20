#pragma once

/// @file
/// Template method implementations for constant_vector_source.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace systems {

template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(
    const Eigen::Ref<const VectorX<T>>& source_value)
    : SingleOutputVectorSource<T>(source_value.rows()),
      source_value_(source_value) {}

template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(const T& source_value)
    : ConstantVectorSource(Vector1<T>::Constant(source_value)) {}

template <typename T>
ConstantVectorSource<T>::~ConstantVectorSource() = default;

template <typename T>
void ConstantVectorSource<T>::DoCalcVectorOutput(
    const Context<T>& context, Eigen::VectorBlock<VectorX<T>>* output) const {
  *output = source_value_;
}

}  // namespace systems
}  // namespace drake
