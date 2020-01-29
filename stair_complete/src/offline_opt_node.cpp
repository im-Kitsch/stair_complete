//
// Created by zhiyuan on 09.11.19.
//

#include <ros/ros.h>

#include <iostream>

#include "std_msgs/Float32.h"

#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/io/layer_io.h>

#include <dynamic_reconfigure/server.h>
#include <stair_complete/offline_dyn_paraConfig.h>

#include "stair_completion.h"


int main(int argc,char **argv){
    ros::init(argc, argv, "offline_opt");
    dynamic_reconfigure::Server<stair_complete::offline_dyn_paraConfig> server;
    dynamic_reconfigure::Server<stair_complete::offline_dyn_paraConfig>::CallbackType f;

    StairComplete st_complete;
    f = boost::bind(&StairComplete::dyn_callback, &st_complete, _1, _2);

    server.setCallback(f);
    ROS_INFO("ok");
    ros::spin();
    return 0;
}
