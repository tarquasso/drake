#include "drake/examples/SoftPaddle/soft_paddle_poincare_map-inl.h"

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

  std::cout << std::setprecision(9);

//  double xn = 0.35; //meters
 // double zn = 0.4; //meters
  //double xdotn = 0.0; // -0.005;   // fixed point in meters on z axis

  // dt = 1e-4        (no filter)           (with filter, tau = 0.15)
  //paddle_aim        0.0002818761488578    0.0508380683037560
  //stroke_strength   0.0734681887024423    0.1229300499221768
  // dt = 1e-3        (no filter)           (with filter, tau = 0.15)
  //paddle_aim        0.0004573758964659    0.0495407071067140
  //stroke_strength   0.0708752914692881    0.1190239261815963
  //double paddle_aim = 0.0495407071067140;
  //double stroke_strength = 0.1190239261815963;

  //double xn = 0.5;
  //double zn = 0.4;
  //double paddle_aim = -0.188732159402915; //0.0; //- 2.0 * M_PI / 180.0;
  //double stroke_strength = 0.0348347361926187; //0.05;

  //double xn = 0.35;
  //double zn = 0.4;
  double xn = 0.418; // fixed point in meters on x axis
  double zn = 0.3;   // fixed point in meters on z axis
  double xdotn = 0.0; // -0.005;   // fixed point in meters on z axis
//
//  //              dt =    1e-3                1e-4
  double paddle_aim = -0.0;//-0.2508743456482843;
  double stroke_strength = 0.1;//0.0266432387875092;

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

  double time_step = 1.0e-4;
  bool filter_command_angle = true; //is that filter needed?

  // Fixed point
  {
    using AutoDiffScalar = Eigen::AutoDiffScalar<Eigen::Vector3d>; //that works for uk but not x
    SoftPaddlePoincareMap<AutoDiffScalar> poincare_map(
        time_step, filter_command_angle);

    AutoDiffScalar paddle_aim_d(paddle_aim, Eigen::Vector3d::UnitX());
    AutoDiffScalar stroke_strength_d(stroke_strength, Eigen::Vector3d::UnitY());
    AutoDiffScalar xdot_d(xdotn, Eigen::Vector3d::UnitZ());

    // Target
    Vector3<double> x0(xn, zn, xdotn);

    PRINT_VAR(paddle_aim_d.value());
    PRINT_VAR(paddle_aim_d.derivatives().transpose());

    PRINT_VAR(stroke_strength_d.value());
    PRINT_VAR(stroke_strength_d.derivatives().transpose());

    int n_iters = 0;
    double relaxation = 0.9; // 0.05
    double tolerance = 1.0e-6;
    Vector3<double> xk;
    Vector3<double> pk(paddle_aim, stroke_strength, xdotn), pkp;
    double error;

    //std::ofstream log("iterations.log");
    
    do {
      std::cout << "--------------------------------------------" << std::endl;
      ++n_iters;

      PRINT_VAR(paddle_aim_d.value());
      PRINT_VAR(stroke_strength_d.value());
      PRINT_VAR(xdot_d.value());

      AutoDiffScalar xnext_d, znext_d, xdotnext_d;
      poincare_map.ComputeNextState(
          paddle_aim_d, stroke_strength_d,
          xn, zn, xdot_d, &xnext_d, &znext_d, &xdotnext_d);
      Vector3<AutoDiffScalar> xk(xn, zn, xdot_d);
      Vector3<AutoDiffScalar> R = xk - Vector3<AutoDiffScalar>(xnext_d, znext_d, xdotnext_d);

      Vector3<double> Rvalue(R[0].value(), R[1].value(), R[2].value());
      Eigen::Matrix<double, 3, 3> Jk;
      Jk.row(0) = R[0].derivatives().transpose();
      Jk.row(1) = R[1].derivatives().transpose();
      Jk.row(2) = R[2].derivatives().transpose();

      PRINT_VAR(xnext_d.value());
      PRINT_VAR(znext_d.value());
      PRINT_VAR(xdotnext_d.value());

      PRINT_VAR(Rvalue.transpose());

      // Compute error.
      error = Rvalue.norm();

      if (error < tolerance && n_iters > 1) break;

      // [u0, u1, xdot]
      pkp = pk - (relaxation * Jk.colPivHouseholderQr().solve(Rvalue) );

      //residual[2] = 0.00;
      //ukp = uk - (relaxation * Jk.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(residual) );
      //ukp = uk - (relaxation * Jk.fullPivHouseholderQr().solve(residual) );
      //ukp = uk - (relaxation * Jk.transpose() * residual );

      PRINT_VAR(error);
      PRINT_VAR(Jk);
      PRINT_VAR(pkp.transpose());

      paddle_aim_d.value() = pkp[0];
      stroke_strength_d.value() = std::max(pkp[1], 0.0); //why the max operation here?
      xdot_d.value() = pkp[2];
      pk = pkp;
    } while(n_iters < 50);

    std::cout << std::fixed << std::setprecision(16);
    std::cout << "--------------------------------------------" << std::endl;
    std::cout << "Final solution:" << std::endl;
    PRINT_VAR(pkp.transpose());
    std::cout << "Iterations: " << n_iters << std::endl;
    //std::cout << "Error: " << error << std::endl;

    paddle_aim = pkp[0]; // assign solution for paddle aim gain
    stroke_strength = pkp[1]; //assign solution for stroke strength
    xdotn = pkp[2];
  }

  { /// Compute discrete LQR
    using AutoDiffScalar = Eigen::AutoDiffScalar<Vector5<double>>;
    SoftPaddlePoincareMap<AutoDiffScalar> poincare_map(
        time_step, filter_command_angle);

    std::cout << " -----------------------------------------------------" << std::endl;
    std::cout << " Computing LQR gain matrix:" << std::endl;

    PRINT_VAR(xn);
    PRINT_VAR(zn);
    PRINT_VAR(xdotn);
    PRINT_VAR(paddle_aim);
    PRINT_VAR(stroke_strength);

    // State variables
    AutoDiffScalar x0(xn, Vector5<double>::Unit(0));
    AutoDiffScalar z0(zn, Vector5<double>::Unit(1));
    AutoDiffScalar xdot0(xdotn, Vector5<double>::Unit(2));

    // Control variables
    AutoDiffScalar paddle_aim0(paddle_aim, Vector5<double>::Unit(3));
    AutoDiffScalar stroke_strength0(stroke_strength, Vector5<double>::Unit(4));

    AutoDiffScalar xnext, znext, xdotnext;

    poincare_map.ComputeNextState(
        paddle_aim0, stroke_strength0,
        x0, z0, xdot0, &xnext, &znext, &xdotnext);

    Eigen::Matrix<double, 3, 5> Jk;
    Jk.row(0) = xnext.derivatives().transpose();
    Jk.row(1) = znext.derivatives().transpose();
    Jk.row(2) = xdotnext.derivatives().transpose();


    Eigen::Matrix<double, 3, 3> A = Jk.block<3, 3>(0, 0); //not sure why it is expecting 4 arguments? still compiles...
    Eigen::Matrix<double, 3, 2> B = Jk.block<3, 2>(0, 3); //not sure why it is expecting 4 arguments? It is a static size matrix and it actually still compiles...

    Eigen::EigenSolver<Matrix3<double>> eigen_solver(A);

    PRINT_VAR(xnext.value());
    PRINT_VAR(znext.value());
    PRINT_VAR(A);
    PRINT_VAR(eigen_solver.eigenvalues().cwiseAbs().transpose());
    PRINT_VAR(B);

    Matrix3<double> Q = 100*Matrix3<double>::Identity();
    Matrix2<double> R = 1.0*Matrix2<double>::Identity();
    // shall we add N with 2 x^T N u?

    R(1,1) = 10000.0;

    PRINT_VAR(Q);
    PRINT_VAR(R);

    auto S = math::DiscreteAlgebraicRiccatiEquation(A, B, Q, R);

    PRINT_VAR(S);

    // (R + B^T * S * B)^{-1} * (B^T * S * A)
    Eigen::LLT<Eigen::MatrixXd> R_cholesky(R + B.transpose() * S * B);
    Eigen::Matrix<double, 2, 3> K = R_cholesky.solve(B.transpose() * S * A);

    Matrix3<double> Atilde = A - B * K;
    Eigen::EigenSolver<Matrix3<double>> eigen_solver_tilde(Atilde);
    PRINT_VAR(eigen_solver_tilde.eigenvalues().cwiseAbs().transpose());

    PRINT_VAR(K);
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
