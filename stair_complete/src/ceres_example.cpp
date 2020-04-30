//
// Created by zhiyuan on 29.04.20.
//


#include "ceres/ceres.h"
#include "glog/logging.h"

#include "ceres_example.h"
using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::CauchyLoss;

/*
 * todo explanation of example
 */

// -----------------------------------------------------------

void compute_value_and_jacobian(  const double* r2,
                                  double* value,
                                  double* jacobian){
    Eigen::Matrix<voxblox::FloatingPoint, 3, 1>  pos, gradient;

    *value = r2[0] * r2[0] - r2[1];
    if (!jacobian){
        return;
    } else{
        jacobian[0] = 2 * r2[0];
        jacobian[1] = -r2[1];
        jacobian[2] = 2;
        return;
    }
}

// -----------------------------------------------------------




bool Evaluate(double const* const* parameters,
              double* residuals,
              double** jacobians) const {

    if (!jacobians) {
        compute_value_and_jacobian(parameters[0], residuals, NULL);
    } else {
        compute_value_and_jacobian(parameters[0], residuals, jacobians[0]);
    }
    return true;
}


// -----------------------------------------------------------


Affine2DWithDistortion::Affine2DWithDistortion(const double theta_in[3], Stair_Interpolater4Ceres* interpolater_in) {

    theta_[0] = theta_in[0];
    theta_[1] = theta_in[1];
    theta_[2] = theta_in[2];

    stair_interpolater_ =  interpolater_in;

    compute_distortion.reset(
            new ceres::CostFunctionToFunctor<1, 3>(new InterpolatorToCeresFunction(stair_interpolater_)));
}

template <typename T>
bool Affine2DWithDistortion::operator()(const T* x,
                                        T* residuals) const {
    T q[3];
    q[0] = theta_[0] + x[0];// * theta_[0] + theta_[0] * theta_[1] * theta_[2];
    q[1] = theta_[1] + x[1];// * theta_[1];
    q[2] = theta_[2] + x[2];// * theta_[2];
    T residual;
    (*compute_distortion)(q, residuals);

    return true;
}

// -----------------------------------------------------------


StairOptimizer::StairOptimizer(const Eigen::MatrixXf& mesh, Stair_Interpolater4Ceres* interpolater_in, double* var_start){

//    google::InitGoogleLogging(argv[0]);
//  don't know the meanning of here
// but just copied from template

    stair_interpolater_ = interpolater_in;
    opt_variable = var_start;

    double theta_i[3];
    for(int i=0; i<mesh.rows(); i++){

        theta_i[0] = mesh(i, 0); //although ugly implementation
        theta_i[1] = mesh(i, 1);
        theta_i[2] = mesh(i, 2);

        problem.AddResidualBlock(
                new AutoDiffCostFunction<Affine2DWithDistortion, 1, 3>(
                        new Affine2DWithDistortion(theta_i, stair_interpolater_)),
                new CauchyLoss(0.5), //NULL,
                opt_variable);
    }
    options.max_num_iterations = 100;
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
