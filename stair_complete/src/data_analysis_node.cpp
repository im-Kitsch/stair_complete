//
// Created by zhiyuan on 09.11.19.
//

#include <iostream>

#include <ros/ros.h>

#include "std_msgs/Float32.h"

#include "parameter.h"

#include "tsdf_load_util.h"

#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/io/layer_io.h>


#include <dynamic_reconfigure/server.h>
#include <stair_complete/offline_dyn_paraConfig.h>
#include <visualizer.h>

#include "stair_mesh_util.h"
#include "stair_optimizer.h"

#include "visualizer.h"
#include "csv_util.h"

using namespace Eigen;
using namespace std;

void report_info(MatrixXd& meshgrid_rotated, double pose[3], voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator) //TODO to package and change it
{
    int num_rows = meshgrid_rotated.rows();
    RowVector3d trans_vec;
    trans_vec<<pose[0], pose[1], pose[2];
    MatrixXd meshgrid = meshgrid_rotated;
    meshgrid.rowwise() += trans_vec;

    MatrixXd total_info(num_rows, 8);
    total_info.block(0, 0, num_rows, 3) = meshgrid;

    for (int i = 0; i < num_rows ; ++i) {
        Eigen::Matrix<voxblox::FloatingPoint, 3, 1>  pos, gradient;
        bool rightness_gra, rightness_dis;
        float distance;
        pos<<meshgrid(i,0), meshgrid(i,1), meshgrid(i,2);
        rightness_gra = interpolator.getGradient(pos, &gradient, true);
        rightness_dis = interpolator.getDistance(pos, &distance, true);

        total_info(i, 3) = gradient(0, 0);
        total_info(i, 4) = gradient(1, 0);
        total_info(i, 5) = gradient(2, 0);
        total_info(i, 6) = distance;
        total_info(i, 7) = rightness_dis * rightness_gra;
    }
    cout<<"pose    gradient    distance   reliability"<<endl;
    cout<<total_info<<endl;
    double reliable_point_percent;
    reliable_point_percent = (double) total_info.col(7).sum() / (double) total_info.rows();
    cout<<"reliable points percent  "<<reliable_point_percent<<endl;
    ROS_INFO("reliable points percent");
    ROS_INFO("%f", reliable_point_percent );
}

double get_reliable_point_rate(MatrixXd& meshgrid_rotated, double pose[3], voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator) //TODO to package and change it
{
    int num_rows = meshgrid_rotated.rows();
    RowVector3d trans_vec;
    trans_vec<<pose[0], pose[1], pose[2];
    MatrixXd meshgrid = meshgrid_rotated;
    meshgrid.rowwise() += trans_vec;

    MatrixXd total_info(num_rows, 8);
    total_info.block(0, 0, num_rows, 3) = meshgrid;

    for (int i = 0; i < num_rows ; ++i) {
        Eigen::Matrix<voxblox::FloatingPoint, 3, 1>  pos, gradient;
        bool rightness_gra, rightness_dis;
        float distance;
        pos<<meshgrid(i,0), meshgrid(i,1), meshgrid(i,2);
        rightness_gra = interpolator.getGradient(pos, &gradient, true);
        rightness_dis = interpolator.getDistance(pos, &distance, true);

        total_info(i, 3) = gradient(0, 0);
        total_info(i, 4) = gradient(1, 0);
        total_info(i, 5) = gradient(2, 0);
        total_info(i, 6) = distance;
        total_info(i, 7) = rightness_dis * rightness_gra;
    }

    double reliable_point_percent;
    reliable_point_percent = (double) total_info.col(7).sum() / (double) total_info.rows();

    return reliable_point_percent;
}

Opt_Record dyn_callback(stair_complete::offline_dyn_paraConfig &config, uint32_t level) {
    static Stair_Visualizer stair_vis("/stair_vis");
    static Grad_Visualizer grad_vis("/grad_vis");
    static Stair_Visualizer stair_vis_opted("/stair_vis_opted", 1.0);
    static Grad_Visualizer grad_vis_opted("/grad_vis_opted");
    system("clear");
//    ROS_INFO("call back begin");
//    if (!config.enable_node )
//        return Opt_Record temp;
//////--- read the vxblx layer and initialize interpolator--------------
//////-----------------TODO known bug---- failed to package it------------------------
    float truncation_distance = config.truncation_distance;
    bool publish_mesh = true;
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_ptr;
    //TODO get relative path
    if (voxblox::io::LoadLayer<voxblox::TsdfVoxel>("/home/zhiyuan/catkin_ws/src/stair_complete/saved_layer/big_truncation.vxblx", &layer_ptr)) {
//        cout<<"loaded success"<<endl;
    } else {
        ROS_ERROR_STREAM("Failed to load tsdf from file.");
//        return false;
    }
    std::shared_ptr<voxblox::TsdfMap> tsdf = std::make_shared<voxblox::TsdfMap>(layer_ptr);
    loadTsdf(tsdf, truncation_distance, publish_mesh);
    voxblox::Interpolator<voxblox::TsdfVoxel> interpolator(tsdf->getTsdfLayerPtr());
////---------------------------------------------------

    Parameter para(config);
//    para.report_para();

    stair_vis.pub_pose(config.stair_length, config.stair_width, config.stair_height, config.stair_num,
            para.stair_init_pos, para.quad);

    //TO Package as function
    {
        MatrixXd mesh_actual;
        MatrixXd grad, reliability;
//        cout<<para.mesh_base<<endl;
        stair_mesh_base2actual(para.mesh_base, mesh_actual, para.stair_init_pos, para.quad);
        grad_cal(mesh_actual, interpolator, grad, reliability);
        grad_vis.pub_grad_pose_grad(mesh_actual, grad, reliability);
    }

    Stair_Interpolater4Ceres stair_interpolator4ceres;
    stair_interpolator4ceres.set_interpolator(&interpolator);

    MatrixXd mesh_rotated;
    stair_mesh_base2rotation(para.mesh_base, mesh_rotated, para.quad);

    cout<<"before optimization"<<endl;
    report_info(mesh_rotated, para.stair_init_pos, interpolator);

    double init_p_rate, final_p_rate;
    init_p_rate = get_reliable_point_rate(mesh_rotated, para.stair_init_pos, interpolator);
    //TODO check here the change of init pose, seems that also has been changed after optimization
//    cout<<para.stair_init_pos[0]<<endl;

    StairOptimizer stair_optimizer(mesh_rotated.cast <float> (), &stair_interpolator4ceres, para.stair_init_pos);
    stair_optimizer.opt_epoc(config.m_opt_loop_n);

    stair_vis_opted.pub_pose(para.stair_length, para.stair_width, para.stair_height,
            para.stair_num, stair_optimizer.opt_variable, para.quad);

    {
        MatrixXd mesh_actual;
        MatrixXd grad, reliability;
        stair_mesh_base2actual(para.mesh_base, mesh_actual, stair_optimizer.opt_variable, para.quad);
        grad_cal(mesh_actual, interpolator, grad, reliability);
        grad_vis_opted.pub_grad_pose_grad(mesh_actual, grad, reliability);
    }

//    cout<<"after optimization"<<endl;
    report_info(mesh_rotated, stair_optimizer.opt_variable, interpolator);
    final_p_rate = get_reliable_point_rate(mesh_rotated, stair_optimizer.opt_variable, interpolator);
    stair_optimizer.opt_record.init_p_rate =init_p_rate;
    stair_optimizer.opt_record.final_p_rate=final_p_rate;

    return stair_optimizer.opt_record;
//    ROS_INFO("call back ends");
}


int main(int argc,char **argv){

    ros::init(argc, argv, "offline_opt");
    dynamic_reconfigure::Server<stair_complete::offline_dyn_paraConfig> server;
    dynamic_reconfigure::Server<stair_complete::offline_dyn_paraConfig>::CallbackType f;

    stair_complete::offline_dyn_paraConfig config;

    config.stair_num=4;
    config.stair_length=0.864;
    config.stair_width=0.244;
    config.stair_height=0.126;
    config.mesh_resolution=0.24;
    config.init_x=-2.3;
    config.init_y=0.8;
    config.init_z=-0.1;
    config.init_yaw=0.0;
    config.init_pitch=0.0;
    config.init_roll=-155.0;
    config.offset_x=0.;
    config.offset_y=0.;
    config.offset_z=0.;
    config.stop_point=2;
    config.m_opt_loop_n=500;
    config.enable_node=true;
    config.point_generation=2;
    config.truncation_distance=0.;
//    dyn_callback(config, 0);

    MatrixXd test_params = load_csv<MatrixXd>("/home/zhiyuan/catkin_ws/src/stair_complete/data/data.csv");
    cout<<"using data"<<endl;
    cout<<test_params<<endl;

    MatrixXd total_record(test_params.rows(), 11+4);

    for (int i = 0; i < test_params.rows(); ++i) {
        config.stair_length = test_params(i, 0);
        config.stair_width = test_params(i, 1);
        config.stair_height = test_params(i, 2);
        config.mesh_resolution = test_params(i, 3);

        Opt_Record my_record =  dyn_callback(config, 0);
        total_record(i, 0) = my_record.init_x;
        total_record(i, 1) = my_record.init_y;
        total_record(i, 2) = my_record.init_z;
        total_record(i, 3) = my_record.final_x;
        total_record(i, 4) = my_record.final_y;
        total_record(i, 5) = my_record.final_z;
        total_record(i, 6) = my_record.init_cost;
        total_record(i, 7) = my_record.final_cost;
        total_record(i, 8) = my_record.opt_steps;
        total_record(i, 9) = my_record.init_p_rate;
        total_record(i, 10) = my_record.final_p_rate;

        total_record(i, 11) = test_params(i, 0);;
        total_record(i, 12) = test_params(i, 1);;
        total_record(i, 13) = test_params(i, 2);;
        total_record(i, 14) = test_params(i, 3);;
    }
    cout<<total_record<<endl;
    writeToCSVfile("/home/zhiyuan/catkin_ws/src/stair_complete/data/result.csv", total_record);
}
