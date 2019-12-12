
#include <iostream>

#include <ros/ros.h>

#include "std_msgs/Float32.h"


#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/io/layer_io.h>

#include "handle_pointcloud.h"
#include "stair_optimizer_util.h"
#include "visualizer.h"
#include "stair_optimizer.h"


#include <dynamic_reconfigure/server.h>
#include <stair_detection/offline_dyn_paraConfig.h>

using namespace Eigen;
using namespace std;


void manually_opt(int loop_n, double opt_step, Grad_Trajectory &grad_traj_pub, Parameters &parameters, MatrixXf &meshgrid_original, voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator){

    MatrixXf pos_total(loop_n, 3), grad_total(loop_n, 3), cost_total(loop_n, 1), ref_pos_total(loop_n, 3);

    for (int i_count = 0; i_count < loop_n; ++i_count) {
        MatrixXf total_info;
        MatrixXf grad_summaried;

        grad_cal(parameters, meshgrid_original, total_info, interpolator);

        grad_summary(total_info, grad_summaried);

        //get middle point for stairs
        MatrixXf center_pos(1, 3);
        center_pos(0,0) = parameters.stair_params[0] / 2;
        center_pos(0,1) = parameters.stair_params[1] * parameters.stair_params[3] / 2;
        center_pos(0,2) = parameters.stair_params[2] * parameters.stair_params[3] / 2;
        // TODO to rewrite the dirty version
        meshgrid_rotation(&parameters, center_pos);
        MatrixXf pos(1, 3);
        pos << parameters.stair_pos[0], parameters.stair_pos[1], parameters.stair_pos[2];
        pos += center_pos;


        MatrixXf grad_summaried_normalized =  opt_step * grad_summaried.normalized();
        pos_total.block(i_count, 0, 1, 3) = pos;
        grad_total.block(i_count, 0, 1, 3) = grad_summaried_normalized;

        int info_rows = total_info.rows();

        VectorXf grad_belief = total_info.block(0, 6, info_rows, 1 );
        VectorXf dis =  total_info.block(0, 7, info_rows, 1 );
        VectorXf dis_belief = total_info.block(0, 8, info_rows, 1 );
        VectorXf dis_discounted = dis.array() * dis_belief.array() * grad_belief.array();

        cost_total(i_count, 0) = dis_discounted.sum();

        // ----------- update parameter
        MatrixXf new_ref_pos(1, 3);
        new_ref_pos << parameters.stair_pos[0], parameters.stair_pos[1], parameters.stair_pos[2];
        new_ref_pos -= grad_summaried_normalized;
        parameters.stair_pos[0] = new_ref_pos(0, 0);
        parameters.stair_pos[1] = new_ref_pos(0, 1);
        parameters.stair_pos[2] = new_ref_pos(0, 2);
        ref_pos_total.block(i_count, 0, 1, 3) = new_ref_pos;
    }
    MatrixXf mat_stack(loop_n, 7) ;
    mat_stack << ref_pos_total, grad_total, cost_total;
    grad_traj_pub.traj_pub_add(pos_total, grad_total);
    cout<<"    ---------grad---summary------------"<<endl;
    cout<<"    pos        grad     err"<<endl;
    cout<<mat_stack<<endl;
}


int opt_structure(stair_detection::offline_dyn_paraConfig &config, uint32_t level){

//////--- read the vxblx layer and initialize interpolator--------------
//////-----------------TODO known bug---- failed to package it------------------------
    float truncation_distance = 0.5f;
    bool publish_mesh = true;
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_ptr;
    if (voxblox::io::LoadLayer<voxblox::TsdfVoxel>("/home/zhiyuan/catkin_ws/src/stair_detection/saved_layer/big_truncation.vxblx", &layer_ptr)) {
        cout<<"loaded success"<<endl;
    } else {
        ROS_ERROR_STREAM("Failed to load tsdf from file.");
        return false;
    }
    std::shared_ptr<voxblox::TsdfMap> tsdf = std::make_shared<voxblox::TsdfMap>(layer_ptr);
    loadTsdf(tsdf, truncation_distance, publish_mesh);
    voxblox::Interpolator<voxblox::TsdfVoxel> interpolator(tsdf->getTsdfLayerPtr());
////---------------------------------------------------

////-----------------------------------------------------------
//// TODO:improve this part  know bug, has to wait for a while then it could actually publish
    static bool initialized = false;

    static Grad_Publisher grad_publisher;
    static Grad_Publisher2 grad_publisher2;
    static Stair_Publisher stair_publisher;
    static Grad_Trajectory grad_traj_pub;
    static Grad_To_Filter_Publisher grad_to_filter_publisher;

    if (!initialized ){
        initialized = true;
        ros::Duration(0.5).sleep();
    }

//// -------------------initialize the parameters------------------------------------

    Parameters parameters;
    //  # calculate the meshgrid
    MatrixXf meshgrid_original;
    stair2meshgrid(parameters.stair_params,  parameters.step,  meshgrid_original );

////---------------------------------------------------------------------------

//---------- self check ----------
//---------visualize meshgrid, meshgrid gradient and stair position
    stair_publisher.pub_pose_euler_radian(parameters.stair_pos[0], parameters.stair_pos[1],
                                   parameters.stair_pos[2], parameters.stair_pos[3],
                                   parameters.stair_pos[4], parameters.stair_pos[5]);

    if (config.stop_point==0){
        point_check(grad_publisher, parameters, meshgrid_original);
        return 0;
    }

    if (config.stop_point==1){
        grad_check(grad_publisher, parameters, meshgrid_original, interpolator, true);
        return 0;
    }

    if (config.stop_point==2){
        grad_filter_check(grad_publisher2, grad_to_filter_publisher, parameters, meshgrid_original, interpolator, true);
        return 0;
    }

    if (config.stop_point==3){
        cout<<"mode 3"<<endl;
        double opt_step = config.m_opt_step;
        int loop_n = config.m_opt_loop_n;
        bool m_opt_del_old_visual = config.m_opt_del_old_visual;
        if(m_opt_del_old_visual) grad_traj_pub.trajectory_reset();

        manually_opt(loop_n, opt_step, grad_traj_pub, parameters, meshgrid_original, interpolator);
        return 0;
    }

    if (config.stop_point==4){
        cout<<"mode 4"<<endl;
        double stair_pos[3] = {parameters.stair_pos[0], parameters.stair_pos[1], parameters.stair_pos[2]};
        MatrixXf meshgrid_rotated = meshgrid_original;
//        meshgrid_rotation(&parameters, meshgrid_rotated);

        Stair_Interpolater4Ceres stair_interpolator4ceres;
        stair_interpolator4ceres.set_interpolator(&interpolator);
        StairOptimizer stair_optimizer(meshgrid_rotated, &stair_interpolator4ceres, stair_pos);

        stair_optimizer.opt_epoc();

        return 0;
    }
    if (config.stop_point==15){
//        MatrixXf total_info;
//        grad_cal(parameters, meshgrid_original, total_info, interpolator);
//        cout<<total_info<<endl;
//
//        MatrixXf grad_summaried;
//        grad_summary(total_info, grad_summaried);

        return 0;
    }

//
////  ---------- ceres --------------
////  optimizer initialization.....
//    double stair_pos[3] = {parameters.stair_pos[0], parameters.stair_pos[1], parameters.stair_pos[2]};
//    MatrixXf meshgrid_rotated = meshgrid_original;
//    meshgrid_rotation(&parameters, meshgrid_rotated);
//
//    Stair_Interpolater4Ceres stair_interpolator4ceres;
//    stair_interpolator4ceres.set_interpolator(&interpolator);
//    StairOptimizer stair_optimizer(meshgrid_rotated, &stair_interpolator4ceres, stair_pos);
//
//    stair_optimizer.opt_epoc();
}

//TODO rethink the logic of parameter setting
void remap_dynPara(stair_detection::offline_dyn_paraConfig &config, uint32_t level){
    ROS_INFO("Reconfigure Request: %d %f %f %f %f",
             config.stair_num,
             config.stair_length,
             config.stair_width,
             config.stair_height,
             config.init_x);

// An example to read private para /opt_offline/xxxx
//    ros::NodeHandle nh("~"); // TODO concern here, maybe problematic, if remap node name
//    nh.getParam("stair_num", st_num);

    ros::param::set("/stair_num", config.stair_num);
    ros::param::set("/stair_height", config.stair_height);
    ros::param::set("/stair_width", config.stair_width);
    ros::param::set("/stair_length", config.stair_length);
    ros::param::set("/init_x", config.init_x);
    ros::param::set("/init_y", config.init_y);
    ros::param::set("/init_z", config.init_z);
    ros::param::set("/init_yaw", config.init_yaw);
    ros::param::set("/init_pitch", config.init_pitch);
    ros::param::set("/init_roll", config.init_roll);
    ros::param::set("/mesh_resolution", config.mesh_resolution);
}





void callback(stair_detection::offline_dyn_paraConfig &config, uint32_t level) {
    remap_dynPara(config, level);
    opt_structure(config, level);
//    offline_optimization_temporal();
}


int main(int argc,char **argv){
    ros::init(argc, argv, "offline_opt");

//    Grad_Trajectory grad_traj_pub;
//    MatrixXf grad1(3, 3), grad2(4, 3), pos1(3, 3), pos2(4, 3);
//    grad1<<-0.5, 0, 0,
//            -0.5, 0, 0,
//            0, 0.5, 0;
//    grad2<<-0.5, 0, 0,
//            0, 0.5, 0,
//            0, 0, 0.5,
//            0, 0, 1;
//    pos1<< 0, 0, 0,
//           -0.5, 0, 0,
//           -1, 0, 0;
//    pos2<< -1, 0.5, 0,
//           -1.5, 0.5, 0,
//           -1.5, 1, 0,
//           -1.5, 1, 0.5;
//    ros::Duration(0.5).sleep();
//    grad_traj_pub.traj_pub_add(pos1, grad1);
//    ros::Duration(3).sleep();
//    grad_traj_pub.traj_pub_add(pos2, grad2);
//    ros::Duration(4).sleep();
//    grad_traj_pub.trajectory_reset();
//    ros::Duration(4).sleep();
//    grad_traj_pub.traj_pub_add(pos1, grad1);
//    ros::spin();

    dynamic_reconfigure::Server<stair_detection::offline_dyn_paraConfig> server;
    dynamic_reconfigure::Server<stair_detection::offline_dyn_paraConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
