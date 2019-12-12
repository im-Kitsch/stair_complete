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

#include "util.h"



using namespace std;

using namespace Eigen;



class Visualizer{
private:

public:
    Visualizer(void);
};

class Stair_Visualizer{
private:


    visualization_msgs::Marker marker1;
    visualization_msgs::Marker marker2;

    ros::NodeHandle nh;
    ros::Publisher Stair_Pub;
public:
    Stair_Visualizer(std::string const topic= "a", double alpha=0.5);
    void pub_pose(double length, double width, double height, int num, double pose[3], tf2::Quaternion orientation);
};

void Stair_Visualizer::pub_pose(double length, double width, double height, int num, double pose[3],
                                tf2::Quaternion orientation) {

    MatrixXd stair_pose_base1, stair_pose_base2;

    MatrixXd index_vec(num, 1);
    index_vec = ArrayXd::LinSpaced(num, 0, num-1);

    stair_pose_base1 = MatrixXd(num, 3);
    stair_pose_base1.col(0) = MatrixXd::Constant(num, 1, length/2);
    stair_pose_base1.col(1) = width * index_vec;
    stair_pose_base1.col(2) = height * (index_vec + MatrixXd::Constant(num, 1, 0.5));

    stair_pose_base2 = MatrixXd(num, 3);
    stair_pose_base2.col(0) = MatrixXd::Constant(num, 1, length/2);
    stair_pose_base2.col(1) = width * (index_vec + MatrixXd::Constant(num, 1, 0.5));
    stair_pose_base2.col(2) = height * (index_vec + MatrixXd::Constant(num, 1, 1));

    marker1.scale.x = length;
    marker1.scale.y = 0.0001;
    marker1.scale.z = height;

    marker2.scale.x = length;
    marker2.scale.y = width;
    marker2.scale.z = 0.0001;

    MatrixXd rot_mat(3, 3);
    tf2_quad_to_eigen_matrix(orientation, rot_mat);

    MatrixXd pose1, pose2;
    RowVector3d offset;
    offset<<pose[0], pose[1], pose[2];
    pose1 = stair_pose_base1 * rot_mat.transpose() ;
    pose2 = stair_pose_base2 * rot_mat.transpose() ;

    pose1.rowwise() += offset;
    pose2.rowwise() += offset;

    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker marker0;
    marker0.action = visualization_msgs::Marker::DELETEALL;
    markerArray.markers.push_back(marker0);

    for (int i = 0; i < num; ++i) {
        marker1.header.stamp = ros::Time::now();
        marker1.id = i*2 + 1;
        marker1.pose.position.x = pose1(i, 0);
        marker1.pose.position.y = pose1(i, 1);
        marker1.pose.position.z = pose1(i, 2);
        marker1.pose.orientation.x = orientation.x();
        marker1.pose.orientation.y = orientation.y();
        marker1.pose.orientation.z = orientation.z();
        marker1.pose.orientation.w = orientation.w();

        marker2.header.stamp = ros::Time::now();
        marker2.id = i*2 + 2;
        marker2.pose.position.x = pose2(i, 0);
        marker2.pose.position.y = pose2(i, 1);
        marker2.pose.position.z = pose2(i, 2);
        marker2.pose.orientation.x = orientation.x();
        marker2.pose.orientation.y = orientation.y();
        marker2.pose.orientation.z = orientation.z();
        marker2.pose.orientation.w = orientation.w();

        markerArray.markers.push_back(marker1);
        markerArray.markers.push_back(marker2);
    }

    Stair_Pub.publish(markerArray);
}

Stair_Visualizer::Stair_Visualizer(std::string topic, double alpha) {

    Stair_Pub = nh.advertise<visualization_msgs::MarkerArray>(topic, 40, true);

    marker1.header.frame_id="/odom";  //TODO to change as parameter
    marker2.header.frame_id="/odom";

    marker1.ns = "basic_shapes";
    marker2.ns = "basic_shapes";

    marker1.type = visualization_msgs::Marker::CUBE;
    marker1.action = visualization_msgs::Marker::ADD;

    marker2.type = visualization_msgs::Marker::CUBE;
    marker2.action = visualization_msgs::Marker::ADD;

    marker1.color.a = alpha;
    marker1.color.r = 120;
    marker1.color.g = 0;
    marker1.color.b = 120;

    marker2.color.a = alpha;
    marker2.color.r = 120;
    marker2.color.g = 120;
    marker2.color.b = 0;
}



class Grad_Visualizer{
public:
    Grad_Visualizer(std::string topic);
    void pub_grad_pose_pose(MatrixXd &meshgrid);
    void pub_grad_pose_grad(const MatrixXd &pose, const MatrixXd &grad, const MatrixXd& reliability);

    ros::NodeHandle nh;
    ros::Publisher markerArrayPub;
    visualization_msgs::Marker marker;
};

Grad_Visualizer::Grad_Visualizer(std::string topic) {
    markerArrayPub = nh.advertise<visualization_msgs::MarkerArray>(topic, 40, true);

    marker.header.frame_id="/odom";  //TODO to change as parameter
    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD; // Attention here!

    marker.scale.x = 0.005;
    marker.scale.y = 0.04;
    marker.scale.z = 0.005;
}

void Grad_Visualizer::pub_grad_pose_grad(const MatrixXd &pose, const MatrixXd &grad, const MatrixXd& reliability){
    MatrixXd pose_end;
    pose_end = pose - grad;
    MatrixXd total_info(pose.rows(), 7);
    total_info.block(0, 0, pose.rows(), 3) = pose;
    total_info.block(0, 3, pose.rows(), 3) = pose_end;
    total_info.block(0,6, pose.rows(), 1) = reliability;
    pub_grad_pose_pose(total_info);
}

void Grad_Visualizer::pub_grad_pose_pose(MatrixXd &meshgrid) {
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker marker_del;
    marker_del.action = visualization_msgs::Marker::DELETEALL;
    markerArray.markers.push_back(marker_del);

    for (unsigned long i = 0; i < meshgrid.rows() ; ++i) {
        marker.id = i+1;
        marker.header.stamp = ros::Time::now();

        geometry_msgs::Point point1, point2;
        point1.x = meshgrid(i, 0);
        point1.y = meshgrid(i, 1);
        point1.z = meshgrid(i, 2);
        point2.x = meshgrid(i, 3);
        point2.y = meshgrid(i, 4);
        point2.z = meshgrid(i, 5);

        marker.points.clear();
        marker.points.push_back(point1);
        marker.points.push_back(point2);

        if (meshgrid(i, 6) == 0){
            marker.color.a = 1;
            marker.color.r = 88;
            marker.color.g = 88;
            marker.color.b = 88;
        } else if(meshgrid(i, 6)==1){
            marker.color.a = 1;
            marker.color.r = 255;
            marker.color.g = 0;
            marker.color.b = 0;
        }
        markerArray.markers.push_back(marker);
    }
    markerArrayPub.publish(markerArray);
}

#endif //STAIR_COMPLETE_VISUALIZER_H
