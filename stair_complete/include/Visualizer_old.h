//
// Created by zhiyuan on 08.09.19.
//
#ifndef STAIR_DETECTION_VISUALIZER_H
#define STAIR_DETECTION_VISUALIZER_H

#include <iostream>


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

#include "handle_pointcloud.h"
#include "stair_optimizer_util.h"


using namespace std;

using namespace Eigen;


class Stair_Visualization{
public:
    Stair_Visualization(int num_in, double length_in, double width_in, double height_in);
    void visualizer_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
    void pub(const geometry_msgs::PoseStamped::ConstPtr& global_pose);
    void visualizer_callback_euler(const geometry_msgs::TwistStamped::ConstPtr& trans_euler);

    void reload_stair(void);

    ros::NodeHandle nh;
    ros::Publisher markerArrayPub;
    ros::Subscriber stair_info_sub;
    ros::Subscriber stair_info_sub_euler;
    ros::Publisher stair_info_pub_euler;

    visualization_msgs::MarkerArray markerArray;

    double height, width, length;
    int number;
    int id_offset = 0;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker2;
};

/*
This is to publish arrow for the gradient.
Example is in main function
Quick test:
    run the node and in terminal run:
     $rostopic pub -1 /visualizer/grad_pose geometry_msgs/PoseArray "{header: auto, poses: [{position: [0,0,0]}, {position: [1,1,1]}]}"
Topics:
Subscribe "/visualizer/grad_pose"
          geometry_msgs::PoseArray
         only pose not orientation is used.
         this topic shows start and end point of the gradient arrow
         Wrongly use oriention, x, y, z, is the gradient vector, w is reliable or not.
Publish "GradArray" visualization_msgs::MarkerArray
        para info http://wiki.ros.org/rviz/DisplayTypes/Marker
*/
class Grad_Visualization{
public:
    Grad_Visualization();
    void visualizer_callback(const geometry_msgs::PoseArray::ConstPtr& pose);
    void pub(const geometry_msgs::PoseArray::ConstPtr& global_pose);

    ros::NodeHandle nh;
    ros::Publisher markerArrayPub;
    ros::Subscriber stair_info_sub;


    int id_offset = 0;
    visualization_msgs::Marker marker;
};

class Grad_Publisher{
public:
    Grad_Publisher(void);

    void pointweise_pub_pseudo(MatrixXf &meshgrid);
    void gradient_pub(MatrixXf &meshgrid);
    ros::NodeHandle n;
    ros::Publisher chatter_pub;
};

class Grad_Publisher2{
public:
    Grad_Publisher2(void);

    void pointweise_pub_pseudo(MatrixXf &meshgrid);
    void gradient_pub(MatrixXf &meshgrid);
    ros::NodeHandle n;
    ros::Publisher chatter_pub;
};

class Grad_To_Filter_Publisher{
public:
    Grad_To_Filter_Publisher(void);
    void pub_grad(MatrixXf &grads);

    ros::NodeHandle n;
    ros::Publisher chatter_pub;
};

void point_check(Grad_Publisher &grad_publisher, Parameters &parameters, MatrixXf &meshgrid_original);
void grad_check(Grad_Publisher &grad_publisher, Parameters &parameters, MatrixXf &meshgrid_original, voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator, bool mat_out);
void grad_filter_check(Grad_Publisher2 &grad_publisher, Grad_To_Filter_Publisher &grad_to_filter_publisher,
                       Parameters &parameters, MatrixXf &meshgrid_original, voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator, bool mat_out);

class Stair_Publisher{
public:
    Stair_Publisher(void);
    void pub_pose_euler_radian(double x, double y, double z, double yaw, double pitch, double roll);

    ros::NodeHandle n;
    ros::Publisher chatter_pub;

};

class Grad_Trajectory{
public:
    Grad_Trajectory(void); //TODO: add input parameter, template marker, topic
    void trajectory_reset(void);
    void traj_pub_add(MatrixXf &pos, MatrixXf &grad);

    ros::NodeHandle n;
    ros::Publisher chatter_pub;

    visualization_msgs::Marker marker_template;
    visualization_msgs::MarkerArray markerArray;
};






#endif //STAIR_DETECTION_VISUALIZER_H
