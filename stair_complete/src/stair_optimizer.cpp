#include <voxblox/core/common.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox/interpolator/interpolator.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/conversions_inl.h>

#include <math.h>
#include <angles/angles.h>

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "stair_optimizer.h"
using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::CauchyLoss;

/*
 * this file mainly optimize the problem
 * min J = min 1/2 * sum_i[ SDF(Rot*vec_p_i+x_trans ) ]
 * optimization variable is x_trans
 * the concrete description is written in the paper
 *
 * To read the code, I highly recommend to firstly read this link
 * http://ceres-solver.org/interfacing_with_autodiff.html
 * The code struct of this file is mostly identical with the example in the link
 */

// -----------------------------------------------------------

void Stair_Interpolater4Ceres::value_and_jacobian(const double* r2,
             double* value,
             double* jacobian){
  Eigen::Matrix<voxblox::FloatingPoint, 3, 1>  pos, gradient;
  float distance;

  pos<< r2[0], r2[1], r2[2] ;


  bool rightness;
  rightness = interpolator_->getDistance(pos, &distance, true);

//  *value = rightness? (double) distance: 0;
  *value =  (double) distance;

  if (!jacobian){
    return;
  }
  else {
      //TODO to check if normalize gradient
    interpolator_->getGradient(pos, &gradient, true);
    jacobian[0] = gradient(0,0);
    jacobian[1] = gradient(1,0);
    jacobian[2] = gradient(2,0);
//    std::cout<<r2[0]<<"  "<<r2[1]<<"  "<<r2[2]<<"  "<<distance<<" "<<*value <<"  "<<jacobian[0]<<"  "<<jacobian[1]<<"  "<<jacobian[2]<<"  "<<std::endl;
    return;
  }
}
void Stair_Interpolater4Ceres::set_interpolator(voxblox::Interpolator<voxblox::TsdfVoxel>* interpolator_in){
  interpolator_ = interpolator_in;
}
void Stair_Interpolater4Ceres::test(){   }
voxblox::Interpolator<voxblox::TsdfVoxel>* interpolator_;

// -----------------------------------------------------------


InterpolatorToCeresFunction::InterpolatorToCeresFunction(Stair_Interpolater4Ceres* interpolater_in): stair_interpolater_(interpolater_in) {};


bool InterpolatorToCeresFunction::Evaluate(double const* const* parameters,
                      double* residuals,
                      double** jacobians) const {

  if (!jacobians) {
    stair_interpolater_->value_and_jacobian(parameters[0], residuals, NULL);
  } else {
    stair_interpolater_->value_and_jacobian(parameters[0], residuals, jacobians[0]);
  }
  return true;
}


// -----------------------------------------------------------


Residual_xyz::Residual_xyz(const double ref_pos_in[3], const double ref_rot_in,
                           Stair_Interpolater4Ceres* interpolater_in) {

    ref_rot = ref_rot_in;
    ref_pos[0] = ref_pos_in[0] * cos(ref_rot) - ref_pos_in[1] * sin(ref_rot) ;
    ref_pos[1] = ref_pos_in[0] * sin(ref_rot) + ref_pos_in[1] * cos(ref_rot);
    ref_pos[2] = ref_pos_in[2];

    std::cout<<"mesh"<<ref_pos[0]<<" "<<ref_pos[1]<<" "<<ref_pos[2]<<std::endl;

    stair_interpolater_ =  interpolater_in;

    compute_distortion.reset(
          new ceres::CostFunctionToFunctor<1, 3>(new InterpolatorToCeresFunction(stair_interpolater_)));
}

template <typename T>
bool Residual_xyz::operator()(const T* x,
                T* residuals) const {
  T q[3];
  q[0] = ref_pos[0] + x[0];
  q[1] = ref_pos[1] + x[1];
  q[2] = ref_pos[2] + x[2];
  T residual;
  (*compute_distortion)(q, residuals);

  return true;
}

// -----------------------------------------------------------

Residual_xyz_rot::Residual_xyz_rot(const double ref_pos_in[3],
                                   Stair_Interpolater4Ceres* interpolater_in) {

    ref_pos[0] = ref_pos_in[0];
    ref_pos[1] = ref_pos_in[1];
    ref_pos[2] = ref_pos_in[2];

    stair_interpolater_ =  interpolater_in;

    compute_distortion.reset(
            new ceres::CostFunctionToFunctor<1, 3>(new InterpolatorToCeresFunction(stair_interpolater_)));
}

template <typename T>
bool Residual_xyz_rot::operator()(const T* opt_pos, const T* opt_rot,
                              T* residuals) const {
    T q[3];
    q[0] = ref_pos[0] * cos(opt_rot[0]) - ref_pos[1] * sin(opt_rot[0]) + opt_pos[0];
    q[1] = ref_pos[0] * sin(opt_rot[0]) + ref_pos[1] * cos(opt_rot[0]) + opt_pos[1];
    q[2] = ref_pos[2] + opt_pos[2];
    T residual;
    (*compute_distortion)(q, residuals);

    return true;
}

// -----------------------------------------------------------


StairOptimizer::StairOptimizer(const Eigen::MatrixXf& mesh, Stair_Interpolater4Ceres* interpolater_in,
                               double* var_start, double* var_rot_in){

//    google::InitGoogleLogging(argv[0]);
//  don't know the meanning of here
// but just copied from template

  stair_interpolater_ = interpolater_in;
  opt_variable = var_start;
  opt_rot = var_rot_in;

  double theta_i[3];
  for(int i=0; i < mesh.rows(); i++){

    theta_i[0] = mesh(i, 0); //although ugly implementation
    theta_i[1] = mesh(i, 1);
    theta_i[2] = mesh(i, 2);
    if (false){
        problem.AddResidualBlock(
                new AutoDiffCostFunction<Residual_xyz, 1, 3>(
                        new Residual_xyz(theta_i, angles::from_degrees(-155.),  stair_interpolater_)),
                new CauchyLoss(0.5), //NULL,
                opt_variable);
    } else{
        problem.AddResidualBlock(
                new AutoDiffCostFunction<Residual_xyz_rot, 1, 3, 1>(
                        new Residual_xyz_rot(theta_i, stair_interpolater_)),
                new CauchyLoss(0.5), //NULL,
                opt_variable, opt_rot);
    }

  }
//  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
}



void StairOptimizer::opt_epoc(int max_num_ite, bool report){
    double original_st[3];
    original_st[0] = opt_variable[0];
    original_st[1] = opt_variable[1];
    original_st[2] = opt_variable[2];

    options.max_num_iterations = max_num_ite;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    std::cout<<"original "<<" x "<<original_st[0]<<" y "<<original_st[1]<<" z "<<original_st[2]<<std::endl;
    std::cout<<"optimized "<<" x "<<opt_variable[0]<<" y "<<opt_variable[1]<<" z "<<opt_variable[2]<<std::endl;
    std::cout<<"final cost  "<<summary.final_cost<<std::endl;
    std::cout<<"original "<<"optimized "<<"final cost  "<<std::endl
    <<"  "<<original_st[0]<<"  "<<original_st[1]<<"  "<<original_st[2]
    <<"  "<<opt_variable[0]<<"  "<<opt_variable[1]<<"  "<<opt_variable[2]
    <<"  "<<summary.final_cost<<std::endl;

    std::cout<<"final roation"<<angles::to_degrees(opt_rot[0])<<std::endl<<std::endl;
    ROS_INFO("original final init_cost final_cost");
    ROS_INFO("%f %f %f %f %f %f %f %f", original_st[0], original_st[1], original_st[2],
             opt_variable[0], opt_variable[1], opt_variable[2], summary.initial_cost,summary.final_cost);

    std::cout << "finished" << std::endl;

    opt_record.init_x=original_st[0];
    opt_record.init_y=original_st[1];
    opt_record.init_z=original_st[2];

    opt_record.final_x=opt_variable[0];
    opt_record.final_y=opt_variable[1];
    opt_record.final_z=opt_variable[2];
    opt_record.init_cost=summary.initial_cost;
    opt_record.final_cost=summary.final_cost;
    opt_record.opt_steps=summary.num_successful_steps;
}

Opt_Record::Opt_Record() {};
