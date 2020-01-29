#ifndef _STAIR_OPTIMIZER_H
#define _STAIR_OPTIMIZER_H

#include <voxblox/core/common.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox/interpolator/interpolator.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/conversions_inl.h>

#include "ceres/ceres.h"
#include "glog/logging.h"
using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;



class Stair_Interpolater4Ceres{
public:
  void value_and_jacobian(const double* r2,
               double* value,
               double* jacobian);
  void set_interpolator(voxblox::Interpolator<voxblox::TsdfVoxel>* interpolator_in);
  void test();
  voxblox::Interpolator<voxblox::TsdfVoxel>* interpolator_;
};



class InterpolatorToCeresFunction : public ceres::SizedCostFunction<1, 3> {
 public:
  InterpolatorToCeresFunction(Stair_Interpolater4Ceres* interpolater_in);


  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const;
private:
  Stair_Interpolater4Ceres* stair_interpolater_;
};


struct Affine2DWithDistortion {
  Affine2DWithDistortion(const double theta_in[3], Stair_Interpolater4Ceres* interpolater_in) ;

  template <typename T>
  bool operator()(const T* x,
                  T* residuals) const ;

  std::unique_ptr<ceres::CostFunctionToFunctor<1, 3> > compute_distortion;
  double theta_[3];
  Stair_Interpolater4Ceres* stair_interpolater_;
};

class Opt_Record{
public:
    Opt_Record();
    double init_x, init_y, init_z;
    double final_x, final_y, final_z;
    double init_p_rate;
    double final_p_rate;
    double init_cost, final_cost;
    int opt_steps;
    void reset(void){};
};


class StairOptimizer {
public:
  StairOptimizer(const Eigen::MatrixXf& mesh, Stair_Interpolater4Ceres* interpolater_in, double* var_start);

  void opt_epoc(int max_num_ite=100, bool report=true);

  Problem problem;
  Solver::Options options;
  Solver::Summary summary;

  Stair_Interpolater4Ceres* stair_interpolater_;
  double* opt_variable;

  Opt_Record opt_record;
};

#endif
