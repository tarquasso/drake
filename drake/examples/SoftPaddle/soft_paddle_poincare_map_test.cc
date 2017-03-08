#include "drake/common/drake_path.h"
#include "drake/examples/SoftPaddle/soft_paddle_poincare_map.h"

#include <iostream>
#include <iomanip>

#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace examples {
namespace soft_paddle {
namespace {

using std::make_unique;
using Eigen::Isometry3d;
using Eigen::Vector2d;

int do_main(int argc, char* argv[]) {


  //double xn = 0.5;
  //double zn = 0.4;
  //double paddle_aim = -0.188732159402915; //0.0; //- 2.0 * M_PI / 180.0;
  //double stroke_strength = 0.0348347361926187; //0.05;

  double xn = 0.525;
  double zn = 0.4;
  //              dt =    1e-3                1e-4
  double paddle_aim = -0.2518274153695856;//-0.2508743456482843;
  double stroke_strength = 0.0283811081944429;//0.0266432387875092;

  // This one diverges even with the solution from x=0.525 above as guess.
  //double xn = 0.55;
  //double zn = 0.4;
  //double paddle_aim = -0.2508743456482843;
  //double stroke_strength = 0.0266432387875092;

  double xnext, znext;

  {
    SoftPaddlePoincareMap<double> poincare_map;

    poincare_map.ComputeNextSate(
        paddle_aim, stroke_strength,
        xn, zn, &xnext, &znext);

    PRINT_VAR(xnext);
    PRINT_VAR(znext);
  }
  // Test for AutoDiffXd
  {
    SoftPaddlePoincareMap<AutoDiffXd> poincare_map;
    AutoDiffXd paddle_aim_d(paddle_aim, Eigen::Vector2d::UnitX());
    AutoDiffXd stroke_strength_d(stroke_strength, Eigen::Vector2d::UnitY());

    PRINT_VAR(paddle_aim_d.value());
    PRINT_VAR(paddle_aim_d.derivatives().transpose());

    PRINT_VAR(stroke_strength_d.value());
    PRINT_VAR(stroke_strength_d.derivatives().transpose());

    AutoDiffXd xnext_d, znext_d;
    poincare_map.ComputeNextSate(
        paddle_aim_d, stroke_strength_d,
        xn, zn, &xnext_d, &znext_d);

    PRINT_VAR(xnext_d.value());
    PRINT_VAR(xnext_d.derivatives().transpose());

    PRINT_VAR(znext_d.value());
    PRINT_VAR(znext_d.derivatives().transpose());
  }

  // Fixed point
  {
    SoftPaddlePoincareMap<AutoDiffXd> poincare_map;
    AutoDiffXd paddle_aim_d(paddle_aim, Eigen::Vector2d::UnitX());
    AutoDiffXd stroke_strength_d(stroke_strength, Eigen::Vector2d::UnitY());

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

      AutoDiffXd xnext_d, znext_d;
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
      if (error < tolerance) break;

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

  return 0;
}

}  // namespace
}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::soft_paddle::do_main(argc, argv);
}
