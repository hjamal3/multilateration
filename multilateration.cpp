#include "ceres/ceres.h"
#include "glog/logging.h"
// Data generated using the following matlab code.
//n = 10;
//r = [];
//theta = [];
//xymin = 0;
//xymax = 100;
//x=xymin+randn(1,n)*(xymax-xymin);
//y=xymin+randn(1,n)*(xymax-xymin);
//robotx = randn()*30;
//roboty = randn()*30;
//sigma = 0.2;
//d = sqrt((x - robotx).*(x - robotx) + (y - roboty).*(y - roboty)) + randn(1,n)*sigma;

const int kNumObservations = 10;
// xi, yi, di
const double data[] = {
   23.98,  -69.036, -65.155,
  119.21, -161.183,  -2.446,
 -194.88,  102.050,  86.172,
   -7.08, -248.628,  58.117,
 -219.24, -231.928,   7.993,
  -94.85,   41.149,  67.698,
   49.23,  266.688,  63.080,
  271.74,  282.731,  19.065,
  205.30,  125.439, 119.767,
    0.12,   85.773,  74.985
};

// clang-format on
using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

struct PositionResidual {
  PositionResidual(double x, double y, double d) : x_(x), y_(y), d_(d) {}
  template <typename T>
  bool operator()(const T* const x_robot, const T* const y_robot, T* residual) const {
    residual[0] = d_ - sqrt((x_ - x_robot[0])*(x_ - x_robot[0]) + (y_ - y_robot[0])*(y_ - y_robot[0]));
    return true;
  }
 private:
  const double x_;
  const double y_;
  const double d_;
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  // initial location
  double x_robot = -20.73;
  double y_robot = 13.48;
  const double x_robot_init = x_robot;
  const double y_robot_init = y_robot;
  Problem problem;
  for (int i = 0; i < kNumObservations; ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<PositionResidual, 1, 1, 1>(
            new PositionResidual(data[3 * i], data[3 * i + 1], data[3 * i + 2])),
        NULL,
        &x_robot,
        &y_robot);
  }
  Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Initial x: " << x_robot_init << " y: " << y_robot_init << "\n";
  std::cout << "Final   x: " << x_robot << " y: " << y_robot << "\n";
  return 0;
}
