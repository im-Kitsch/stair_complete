//
// Created by zhiyuan on 08.09.19.
//
#ifndef STAIR_COMPLETE_VISUALIZER_H
#define STAIR_COMPLETE_VISUALIZER_H

#include <iostream>
#include <string>

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TwistStamped.h"


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <angles/angles.h>

#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox/io/layer_io.h>


#include "stair_mesh_util.h"
#include "parameter.h"

#include "ceres/ceres.h"
#include "glog/logging.h"


using namespace std;

using namespace Eigen;


class Stair_Visualizer{
private:
    visualization_msgs::Marker marker1;
    visualization_msgs::Marker marker2;

    ros::NodeHandle nh;
    ros::Publisher Stair_Pub;
public:
    Stair_Visualizer(std::string const topic= "a", double alpha=0.5, std::string frame_id="/odom");
    void pub_pose(double length, double width, double height, int num, double pose[3], tf2::Quaternion orientation);
};


class Grad_Visualizer{
public:
    Grad_Visualizer(std::string topic, std::string frame_id="/odom");
    void pub_grad_pose_pose(MatrixXd &meshgrid);
    void pub_grad_pose_grad(const MatrixXd &pose, const MatrixXd &grad, const MatrixXd& reliability);

    ros::NodeHandle nh;
    ros::Publisher markerArrayPub;
    visualization_msgs::Marker marker;
};

void grad_cal(MatrixXd &mesh, voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator,
              MatrixXd& grad, MatrixXd& reliability, bool normalize=false);

void visualize_grad(Parameter para, voxblox::Interpolator<voxblox::TsdfVoxel> interpolator,
                    Grad_Visualizer grad_vis, double pose[3]);


#endif //STAIR_COMPLETE_VISUALIZER_H
