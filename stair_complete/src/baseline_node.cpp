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
#include "stair_completion.h"

#include "visualizer.h"
#include "csv_util.h"

using namespace Eigen;
using namespace std;

void get_grad_info(MatrixXd& meshgrid_rotated, double pose[3], voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator,
              MatrixXd& meshgrid_result)
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
//    std::cout<<"pose    gradient    distance   reliability"<<std::endl;
//    std::cout<<total_info<<std::endl;
//    double reliable_point_percent;
//    reliable_point_percent = (double) total_info.col(7).sum() / (double) total_info.rows();
//    std::cout<<"reliable points percent  "<<reliable_point_percent<<std::endl;

    meshgrid_result = total_info;
    return;
}

int baseline_optimization(stair_complete::offline_dyn_paraConfig &config, uint32_t level){
    system("clear");
    ROS_INFO("call back begin");
    if (!config.enable_node ) return 0;

    //////--- read the vxblx layer and initialize interpolator--------------
    // Function Encapsulation is not implemented in this block
    // If do so, "Segementation error" occurs
    // It seems that the loded data from file is deleted after executing function
    float truncation_distance = config.truncation_distance;
    bool publish_mesh = true;
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_ptr;

    std::string tsdf_path;
    ros::param::get("/tsdf_save_path", tsdf_path);

    if (voxblox::io::LoadLayer<voxblox::TsdfVoxel>(
            tsdf_path,
            &layer_ptr)) {
        cout<<"loaded success"<<endl;
    } else {
        ROS_ERROR_STREAM("Failed to load tsdf from file.");
        return false;
    }
    std::shared_ptr<voxblox::TsdfMap> tsdf = std::make_shared<voxblox::TsdfMap>(layer_ptr);
    loadTsdf(tsdf, truncation_distance, publish_mesh);
    voxblox::Interpolator<voxblox::TsdfVoxel> interpolator(tsdf->getTsdfLayerPtr());
    ////---------------------------------------------------

    static Stair_Visualizer stair_vis("/stair_vis");
    static Grad_Visualizer grad_vis("/grad_vis");
    static Grad_Visualizer grad_vis_opted("/grad_vis_opted");
    static Stair_Visualizer stair_vis_opted("/stair_vis_opted", 1.0);
    // befor and after optimization, the stair visualization has different transparency

    // Note that the ROS-Publisher could not publish information immediately,
    // it publishes successfully after few seconds

    Parameter para(config);
    para.report_para();

    MatrixXd mesh_rotated;
    stair_mesh_base2rotation(para.mesh_base, mesh_rotated, para.quad);

    double st_init_pos[3] = {para.stair_init_pos[0], para.stair_init_pos[1], para.stair_init_pos[2]};

    stair_vis.pub_pose(config.stair_length, config.stair_width, config.stair_height,
                       config.stair_num, st_init_pos, para.quad);

    double learning_rate = 0.01;

    MatrixXd opt_position(1, 3);
    opt_position << para.stair_init_pos[0], para.stair_init_pos[1], para.stair_init_pos[2];

    int opt_steps = 500;
    double stopping_creterion = 0.0000001;

    MatrixXd pos_record(opt_steps, 3), grad_record(opt_steps, 3), realiability_record(opt_steps, 1);

    RowVector3d offset_center;
    vec_tf_from_rotation(offset_center, config.stair_length/2, config.stair_width * config.stair_num / 2.,
                         config.stair_height * config.stair_num / 2., config.init_yaw,
                         config.init_pitch, config.init_roll);

    int i = 0;
    double last_cost = 100.;
    for (i = 0; i < opt_steps; ++i) {
        double position[3] = {opt_position(0, 0), opt_position(0, 1), opt_position(0, 2)};

        MatrixXd info_mat;
        get_grad_info(mesh_rotated, position, interpolator, info_mat);

        MatrixXd grad;
        int num_rows = info_mat.rows();
        grad = info_mat.block(0, 3, num_rows, 3);
        VectorXd distance = info_mat.block(0, 6, num_rows, 1);
        VectorXd reliability = info_mat.block(0, 7, num_rows, 1);

        MatrixXd temp = grad.array().colwise() * reliability.array();
        MatrixXd temp2 = temp.array().colwise() * distance.array();

        MatrixXd sum_grad = temp2.colwise().sum();
        sum_grad *= learning_rate;

        MatrixXd cost_temp = distance.array() * distance.array();

        double cost = cost_temp.sum();

        cost /= 2.;

        cout<<"step "<< i<<" cost "<< cost <<endl;

        pos_record(i, 0) = opt_position(0, 0) + offset_center(0, 0);
        pos_record(i, 1) = opt_position(0, 1) + offset_center(0, 1);
        pos_record(i, 2) = opt_position(0, 2) + offset_center(0, 2);
        grad_record(i, 0) = sum_grad(0, 0);
        grad_record(i, 1) = sum_grad(0, 1);
        grad_record(i, 2) = sum_grad(0, 2);
        realiability_record(i, 0) = 1;

        opt_position -= sum_grad;

        if (abs(last_cost - cost) < stopping_creterion) {
            opt_steps = i;}

        // don't know why use this sentence then it doesn't work
//        if (last_cost - cost < stopping_creterion)
//            break;
        last_cost = cost;
    }
    cout<< endl << "steps" << opt_steps <<endl;
    cout<< endl << "result" << opt_position <<endl;
    grad_vis.pub_grad_pose_grad(pos_record.block(0, 0, i, 3),
            grad_record.block(0, 0, i, 3),
            realiability_record.block(0, 0, i, 1));

    double opted_pos[3] = {opt_position(0, 0), opt_position(0, 1), opt_position(0, 2)};
    stair_vis_opted.pub_pose(config.stair_length, config.stair_width, config.stair_height,
                       config.stair_num, opted_pos, para.quad);

    return 0;
};


int main(int argc,char **argv){

//    ros::init(argc, argv, "offline_opt");
//
//    StairComplete st_complete;
//
//    stair_complete::offline_dyn_paraConfig config;
//
//    config.stair_num=4;
//    config.stair_length=0.864;
//    config.stair_width=0.244;
//    config.stair_height=0.126;
//    config.mesh_resolution=0.24;
//    config.init_x=-2.3;
//    config.init_y=0.8;
//    config.init_z=-0.1;
//    config.init_yaw=0.0;
//    config.init_pitch=0.0;
//    config.init_roll=-155.0;
//    config.offset_x=0.;
//    config.offset_y=0.;
//    config.offset_z=0.;
//    config.m_opt_loop_n=500;
//    config.enable_node=true;
//    config.point_generation=2;
//    config.truncation_distance=0.;
//    config.report_grad = true;
//    config.do_optimization = true;
//
//
//    baseline_optimization(config);
//    ros::spin();

    ros::init(argc, argv, "offline_opt");
    dynamic_reconfigure::Server<stair_complete::offline_dyn_paraConfig> server;
    dynamic_reconfigure::Server<stair_complete::offline_dyn_paraConfig>::CallbackType f;

    StairComplete st_complete;

    f = boost::bind(&baseline_optimization, _1, _2);

    server.setCallback(f);
    ROS_INFO("ok");
    ros::spin();

    return 0;
}
