#include "drake/multibody/multibody_tree/articulated_body_inertia.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"
#include "drake/multibody/multibody_tree/unit_inertia.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Test default constructor which leaves entries initialized to NaN for a
// quick detection of uninitialized state. Also tests CopyToFullMatrix6().
GTEST_TEST(ArticulatedBodyInertia, DefaultConstructor) {
  ArticulatedBodyInertia<double> P;
  const Matrix6<double> P_matrix = P.CopyToFullMatrix6();
  ASSERT_TRUE(std::all_of(
      P_matrix.data(),
      P_matrix.data() + 36,
      [](double x) { return std::isnan(x); }
  ));
}

// Construct a non-trivial articulated body inertia from a spatial inertia,
// testing both construction from matrix and construction from spatial inertia.
GTEST_TEST(ArticulatedBodyInertia, ConstructionNonTrivial) {
  const double mass = 2.5;
  const Vector3d com(0.1, -0.2, 0.3);
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  const UnitInertia<double> G(m(0), m(1), m(2),  /* moments of inertia */
                              p(0), p(1), p(2)); /* products of inertia */
  const SpatialInertia<double> M(mass, com, G);

  // Construct from matrix directly.
  const Matrix6<double> M_matrix = M.CopyToFullMatrix6();
  const ArticulatedBodyInertia<double> P_direct(M_matrix);
  EXPECT_TRUE(P_direct.CopyToFullMatrix6().isApprox(M_matrix, kEpsilon));

  // Construct from spatial inertia (indirectly).
  const ArticulatedBodyInertia<double> P_indirect(M);
  EXPECT_TRUE(P_direct.CopyToFullMatrix6().isApprox(M_matrix, kEpsilon));
}

// Test the shift function by comparing to the spatial inertia shift function.
GTEST_TEST(SpatialInertia, Shift) {
  const double mass = 2.5;
  const Vector3d com(0.1, -0.2, 0.3);
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  const UnitInertia<double> G(m(0), m(1), m(2),  /* moments of inertia */
                              p(0), p(1), p(2)); /* products of inertia */
  const SpatialInertia<double> M(mass, com, G);
  const ArticulatedBodyInertia<double> P(M);

  const SpatialInertia<double> M_shifted = M.Shift(Vector3d::UnitX());
  const ArticulatedBodyInertia<double> P_shifted = P.Shift(Vector3d::UnitX());

  EXPECT_TRUE(P_shifted.CopyToFullMatrix6()
                  .isApprox(M_shifted.CopyToFullMatrix6(), kEpsilon));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
