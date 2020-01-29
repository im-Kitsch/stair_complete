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


int main(int argc,char **argv){

    ros::init(argc, argv, "offline_opt");

    StairComplete st_complete;

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
//    config.stop_point=2;
    config.m_opt_loop_n=500;
    config.enable_node=true;
    config.point_generation=2;
    config.truncation_distance=0.;
    config.report_grad = true;
    config.do_optimization = true;
//    dyn_callback(config, 0);

    std::string data_path;
    ros::param::get("/data_file", data_path);
    MatrixXd test_params = load_csv<MatrixXd>(data_path);
    cout<<"using data"<<endl;
    cout<<test_params<<endl;

    MatrixXd total_record(test_params.rows(), 11+4);

    for (int i = 0; i < test_params.rows(); ++i) {
        config.stair_length = test_params(i, 0);
        config.stair_width = test_params(i, 1);
        config.stair_height = test_params(i, 2);
        config.mesh_resolution = test_params(i, 3);

        Opt_Record my_record;
        st_complete.stair_completion(config, my_record);
//        Opt_Record my_record =  dyn_callback(config, 0);
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
    std::string result_path;
    ros::param::get("/result_file", result_path);
    writeToCSVfile(result_path, total_record);
}
