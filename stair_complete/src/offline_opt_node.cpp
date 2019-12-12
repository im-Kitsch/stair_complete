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
}

int dyn_callback(stair_complete::offline_dyn_paraConfig &config, uint32_t level) {
    static Stair_Visualizer stair_vis("/stair_vis");
    static Grad_Visualizer grad_vis("/grad_vis");
    static Stair_Visualizer stair_vis_opted("/stair_vis_opted", 1.0);
    static Grad_Visualizer grad_vis_opted("/grad_vis_opted");
    system("clear");
    ROS_INFO("call back begin");
    if (!config.enable_node ) return 0;
//////--- read the vxblx layer and initialize interpolator--------------
//////-----------------TODO known bug---- failed to package it------------------------
    float truncation_distance = config.truncation_distance;
    bool publish_mesh = true;
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_ptr;
    //TODO get relative path
    if (voxblox::io::LoadLayer<voxblox::TsdfVoxel>("/home/zhiyuan/catkin_ws/src/stair_complete/saved_layer/big_truncation.vxblx", &layer_ptr)) {
        cout<<"loaded success"<<endl;
    } else {
        ROS_ERROR_STREAM("Failed to load tsdf from file.");
        return false;
    }
    std::shared_ptr<voxblox::TsdfMap> tsdf = std::make_shared<voxblox::TsdfMap>(layer_ptr);
    loadTsdf(tsdf, truncation_distance, publish_mesh);
    voxblox::Interpolator<voxblox::TsdfVoxel> interpolator(tsdf->getTsdfLayerPtr());
////---------------------------------------------------

    Parameter para(config);
    para.report_para();

    stair_vis.pub_pose(config.stair_length, config.stair_width, config.stair_height, config.stair_num,
            para.stair_init_pos, para.quad);

    //TO Package as function
    {
        MatrixXd mesh_actual;
        MatrixXd grad, reliability;
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
    //TODO check here the change of init pose, seems that also has been changed after optimization
    cout<<para.stair_init_pos[0]<<endl;

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

    cout<<"after optimization"<<endl;
    report_info(mesh_rotated, stair_optimizer.opt_variable, interpolator);
//    config.offset_x = 10;
    ROS_INFO("call back ends");
    return 0;
}


int main(int argc,char **argv){
    ros::init(argc, argv, "offline_opt");
    dynamic_reconfigure::Server<stair_complete::offline_dyn_paraConfig> server;
    dynamic_reconfigure::Server<stair_complete::offline_dyn_paraConfig>::CallbackType f;

    f = boost::bind(&dyn_callback, _1, _2);
    server.setCallback(f);
    ROS_INFO("ok");
    ros::spin();
    return 0;
}
