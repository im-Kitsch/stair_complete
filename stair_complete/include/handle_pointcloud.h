#ifndef _HANDEL_POINTCLOUD_H
#define _HANDEL_POINTCLOUD_H

#include <iostream>
#include <typeinfo>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <angles/angles.h>

#include "voxblox/core/common.h"
#include <voxblox_msgs/Layer.h>
#include <voxblox/interpolator/interpolator.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/conversions_inl.h>

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


using namespace Eigen;
using namespace std;



struct Parameters{
    std::string name;                // name of advertised node
    std::string node_path;           // name of subscribed node
    float stair_pos[6];         // x, y, z, yaw, pitch, roll with unit: meter and radian
    float stair_params[4];      //length, width, height, number with unit: meter
    float step;                 // step for dividing the stair model into points, unit: meter
    float edge_len;             //egde length of cubic used for interpolation, prefered equals step
    char* argv;
    Parameters(void){
        this->name = "/handled_pointcloud";
        this->node_path = "/voxblox_node/tsdf_map_out";

        ros::param::param("init_x", this->stair_pos[0], (float)-2.2);
        ros::param::param("init_y", this->stair_pos[1], (float)1.0);
        ros::param::param("init_z", this->stair_pos[2], (float)0.0);
        ros::param::param("init_yaw", this->stair_pos[3], (float)0.0);
        ros::param::param("init_pitch", this->stair_pos[4], (float)0.0);
        ros::param::param("init_roll", this->stair_pos[5], (float)-150.0);

        //degree and radian transformation TODO to improve
        this->stair_pos[3] = angles::from_degrees(this->stair_pos[3]);
        this->stair_pos[4] = angles::from_degrees(this->stair_pos[4]);
        this->stair_pos[5] = angles::from_degrees(this->stair_pos[5]);

        ros::param::param("stair_length", this->stair_params[0], (float)1.164);     // length
        ros::param::param("stair_width", this->stair_params[1], (float)0.244);    // width
        ros::param::param("stair_height", this->stair_params[2], (float)0.126);    // height
        ros::param::param("stair_num", this->stair_params[3], (float)2.0);       // number of stairs

        ros::param::param("mesh_resolution", this->step, (float)0.05);
        this->edge_len = 0.05;
    }
};

class handle_pointcloud   //TODO should be big character, Handle_PointCloud
{
public:
    handle_pointcloud(Parameters parameters);
    void call_back(const voxblox_msgs::Layer& input);
//    voxblox::FloatingPoint residuum_calcu(voxblox::Interpolator<voxblox::TsdfVoxel> interpolator);
//    voxblox::Point jocabian(voxblox::Interpolator<voxblox::TsdfVoxel> interpolator);

    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    Parameters params;
private:
};

#endif

