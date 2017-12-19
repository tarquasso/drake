#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/cross_product.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {

template<typename T>
class ArticulatedInertia {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArticulatedInertia)

  ArticulatedInertia() {}

  explicit ArticulatedInertia(const Matrix6<T>& I_SP_E) : I_SP_E_(I_SP_E) {}

  explicit ArticulatedInertia(const SpatialInertia<T>& M_SP_E)
      : I_SP_E_(M_SP_E.CopyToFullMatrix6()) {}

  Matrix6<T>& get_matrix() { return I_SP_E_; }

  const Matrix6<T>& get_matrix() const { return I_SP_E_; }

  ArticulatedInertia<T> ReExpress(const Matrix3<T>& R_AE) const {
    Matrix6<T> T_EA = CalcShift_(R_AE.transpose(), Vector3<T>::Zero());
    return ArticulatedInertia<T>(T_EA.transpose() * I_SP_E_ * T_EA);
  }

  ArticulatedInertia<T> Shift(const Vector3<T>& p_PQ_E) const {
    Matrix6<T> T_EA = CalcShift_(Matrix3<T>::Identity(), p_PQ_E);
    return ArticulatedInertia<T>(T_EA.transpose() * I_SP_E_ * T_EA);
  }

  ArticulatedInertia<T>& operator+=(const ArticulatedInertia<T>& I_BP_E) {
    I_SP_E_ += I_BP_E.get_matrix();
    return *this;
  }

 private:
  /// See [Springer 2008, Eq. 2.9].
  const Matrix6<T> CalcShift_(const Matrix3<T>& R_FG,
                              const Vector3<T>& p_FoGo_F) const {
    using math::VectorToSkewSymmetric;

    // Construct shift matrix.
    Matrix6<T> T_FG = Matrix6<T>::Zero();

    // Top left and bottom right are R_FG.
    T_FG.block(0, 0, 3, 3) = R_FG;
    T_FG.block(3, 3, 3, 3) = R_FG;

    // Top right is l_FG * R_FG.
    Matrix3<T> l_FG = VectorToSkewSymmetric(p_FoGo_F);
    T_FG.block(3, 0, 3, 3) = l_FG * R_FG;

    return T_FG;
  }

  Matrix6<T> I_SP_E_{};
};

}
}