#include "drake/examples/SoftPaddle/soft_paddle_poincare_map.h"

#include <iostream>
#include <iomanip>

#include "drake/math/discrete_algebraic_riccati_equation.h"

#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace examples {
namespace soft_paddle {
namespace {

using std::make_unique;
using Eigen::Isometry3d;
using Eigen::Vector2d;

int do_main(int argc, char* argv[]) {


  double xn = 0.35;
  double zn = 0.4;
  // dt = 1e-4        (no filter)           (with filter, tau = 0.15)
  //paddle_aim        0.0002818761488578    0.0508380683037560
  //stroke_strength   0.0734681887024423    0.1229300499221768
  // dt = 1e-3        (no filter)           (with filter, tau = 0.15)
  //paddle_aim        0.0004573758964659    0.0495407071067140
  //stroke_strength   0.0708752914692881    0.1190239261815963
  double paddle_aim = 0.0002818761488578;
  double stroke_strength = 0.0734681887024423;

  //double xn = 0.5;
  //double zn = 0.4;
  //double paddle_aim = -0.188732159402915; //0.0; //- 2.0 * M_PI / 180.0;
  //double stroke_strength = 0.0348347361926187; //0.05;

  //double xn = 0.525;
  //double zn = 0.4;
  //              dt =    1e-3                1e-4
  //double paddle_aim = -0.2518274153695856;//-0.2508743456482843;
  //double stroke_strength = 0.0283811081944429;//0.0266432387875092;

  // This one diverges even with the solution from x=0.525 above as guess.
  //double xn = 0.55;
  //double zn = 0.4;
  //double paddle_aim = -0.2508743456482843;
  //double stroke_strength = 0.0266432387875092;

  //double xn = 0.25;
  //double zn = 0.4;
  // dt = 1e-4        (no filter)           (with filter)
  //double paddle_aim = 0.0933383;         // 0.1519477242286417
  //double stroke_strength = 0.0892304;    // 0.1598050387931525

#if 0
  double xnext, znext;
  {
    SoftPaddlePoincareMap<double> poincare_map;

    poincare_map.ComputeNextSate(
        paddle_aim, stroke_strength,
        xn, zn, &xnext, &znext);

    PRINT_VAR(xnext);
    PRINT_VAR(znext);
  }
  // Test for AutoDiffScalar
  {
    SoftPaddlePoincareMap<AutoDiffScalar> poincare_map;
    AutoDiffScalar paddle_aim_d(paddle_aim, Eigen::Vector2d::UnitX());
    AutoDiffScalar stroke_strength_d(stroke_strength, Eigen::Vector2d::UnitY());

    PRINT_VAR(paddle_aim_d.value());
    PRINT_VAR(paddle_aim_d.derivatives().transpose());

    PRINT_VAR(stroke_strength_d.value());
    PRINT_VAR(stroke_strength_d.derivatives().transpose());

    AutoDiffScalar xnext_d, znext_d;
    poincare_map.ComputeNextSate(
        paddle_aim_d, stroke_strength_d,
        xn, zn, &xnext_d, &znext_d);

    PRINT_VAR(xnext_d.value());
    PRINT_VAR(xnext_d.derivatives().transpose());

    PRINT_VAR(znext_d.value());
    PRINT_VAR(znext_d.derivatives().transpose());
  }
#endif

  double time_step = 1.0e-3;
  bool filter_command_angle = false;

  // Fixed point
  {
    using AutoDiffScalar = Eigen::AutoDiffScalar<Eigen::Vector2d>;
    SoftPaddlePoincareMap<AutoDiffScalar> poincare_map(
        time_step, filter_command_angle);
    AutoDiffScalar paddle_aim_d(paddle_aim, Eigen::Vector2d::UnitX());
    AutoDiffScalar stroke_strength_d(stroke_strength, Eigen::Vector2d::UnitY());

    // Target
    Vector2d x0(xn, zn);

    PRINT_VAR(paddle_aim_d.value());
    PRINT_VAR(paddle_aim_d.derivatives().transpose());

    PRINT_VAR(stroke_strength_d.value());
    PRINT_VAR(stroke_strength_d.derivatives().transpose());

    int n_iters = 0;
    double relaxation = 0.9;
    double tolerance = 1.0e-6;
    Vector2<double> xk, uk, ukp;
    double error;
    do {
      std::cout << "--------------------------------------------" << std::endl;
      ++n_iters;

      AutoDiffScalar xnext_d, znext_d;
      poincare_map.ComputeNextSate(
          paddle_aim_d, stroke_strength_d,
          xn, zn, &xnext_d, &znext_d);

      uk << paddle_aim_d.value(), stroke_strength_d.value();
      xk << xnext_d.value(), znext_d.value();
      Matrix2<double> Jk;
      Jk.row(0) = xnext_d.derivatives().transpose();
      Jk.row(1) = znext_d.derivatives().transpose();

      // Compute error.
      error = (xk - x0).norm();
      if (error < tolerance && n_iters > 1) break;

      ukp = uk - relaxation * Jk.colPivHouseholderQr().solve(xk - x0);

      PRINT_VAR(xk.transpose());
      PRINT_VAR(error);
      //PRINT_VAR(Jk);
      //PRINT_VAR(ukp.transpose());

      paddle_aim_d.value() = ukp[0];
      stroke_strength_d.value() = std::max(ukp[1], 0.0);

    } while(n_iters < 50);

    std::cout << std::fixed << std::setprecision(16);
    std::cout << "--------------------------------------------" << std::endl;
    std::cout << "Final solution:" << std::endl;
    PRINT_VAR(ukp.transpose());
    std::cout << "Iterations: " << n_iters << std::endl;
    std::cout << "Error: " << error << std::endl;

  }
#if 0
  { /// Compute discrete LQR
    using AutoDiffScalar = Eigen::AutoDiffScalar<Eigen::Vector4d>;
    SoftPaddlePoincareMap<AutoDiffScalar> poincare_map;

    // State variables
    AutoDiffScalar x0(xn, Eigen::Vector4d::Unit(0));
    AutoDiffScalar z0(zn, Eigen::Vector4d::Unit(1));

    // Control variables
    AutoDiffScalar paddle_aim0(paddle_aim, Eigen::Vector4d::Unit(2));
    AutoDiffScalar stroke_strength0(stroke_strength, Eigen::Vector4d::Unit(3));

    AutoDiffScalar xnext, znext;

    poincare_map.ComputeNextSate(
        paddle_aim0, stroke_strength0,
        x0, z0, &xnext, &znext);

    Eigen::Matrix<double, 2, 4> Jk;
    Jk.row(0) = xnext.derivatives().transpose();
    Jk.row(1) = znext.derivatives().transpose();

    Matrix2<double> A = Jk.block<2, 2>(0, 0);
    Matrix2<double> B = Jk.block<2, 2>(0, 2);

    PRINT_VAR(xnext.value());
    PRINT_VAR(znext.value());
    PRINT_VAR(A);
    PRINT_VAR(B);

    Matrix2<double> Q = Matrix2<double>::Identity();
    Matrix2<double> R = Matrix2<double>::Identity();

    PRINT_VAR(Q);
    PRINT_VAR(R);

    auto S = math::DiscreteAlgebraicRiccatiEquation(A, B, Q, R);

    PRINT_VAR(S);

    Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
    Matrix2<double> K = R_cholesky.solve(B.transpose() * S);

    PRINT_VAR(K);
  }
#endif
  return 0;
}

}  // namespace
}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::soft_paddle::do_main(argc, argv);
}
