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
// for i = 1:length(x)
//    fprintf('%8.2f, %8.3f,%8.3f,\n',[x(i),y(i),d(i)]')   
//end 
const int kNumObservations = 10;
// xi, yi, di
const double data[] = {
  -86.49,   -0.685, 132.556,
   -3.01,  153.263, 158.406,
  -16.49,  -76.967, 101.143,
   62.77,   37.138,  38.743,
  109.33,  -22.558,  67.707,
  110.93,  111.736, 126.991,
  -86.37, -108.906, 173.270,
    7.74,    3.256,  38.774,
 -121.41,   55.253, 175.661,
 -111.35,  110.061, 190.545
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
  double x_robot = 0.0;
  double y_robot = 0.0;
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
