//
// Created by zhiyuan on 08.11.19.
//

#ifndef STAIR_COMPLETE_PARAMETER_H
#define STAIR_COMPLETE_PARAMETER_H

#include <iostream>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <angles/angles.h>

#include <stair_complete/offline_dyn_paraConfig.h>
#include <tf2/LinearMath/Matrix3x3.h>

//#include "ceres/ceres.h"
#include "Eigen/Core"

#include "stair_mesh_util.h"

using namespace Eigen;
using namespace std;

//change frame //临时函数
void vec_tf_from_rotation(RowVector3d& offset_vec, double x, double y, double z,
                        double yaw, double pitch, double roll){

    tf2::Quaternion quad;
    quad.setEuler(
            angles::from_degrees(yaw),
            angles::from_degrees(pitch),
            angles::from_degrees(roll)
    );

    tf2::Matrix3x3 rot_matrix(quad);
    Matrix3d rot_mat;
    rot_mat <<  rot_matrix.getRow(0)[0], rot_matrix.getRow(0)[1], rot_matrix.getRow(0)[2],
            rot_matrix.getRow(1)[0], rot_matrix.getRow(1)[1], rot_matrix.getRow(1)[2],
            rot_matrix.getRow(2)[0], rot_matrix.getRow(2)[1], rot_matrix.getRow(2)[2];
    Vector3d offset_col_vec;
    offset_col_vec<<x, y, z;
    offset_col_vec = rot_mat * offset_col_vec;
    offset_vec(0, 0) = offset_col_vec(0,0);
    offset_vec(0, 1) = offset_col_vec(1,0);
    offset_vec(0, 2) = offset_col_vec(2,0);
}

class Parameter{
private:

public:
    double stair_init_pos[3]; // initial position x, y, z TODO To add sure which cm or m
    double mesh_resolution; // TODO to declare unit cm or m
    double stair_length, stair_width, stair_height;
    int stair_num;
    double truncation_distance;
    tf2::Quaternion quad;
    MatrixXd mesh_base;

    Parameter(int argc,char **argv); // Not used and not tested
    Parameter(void); // Not used and not tested
    Parameter(stair_complete::offline_dyn_paraConfig &config); // used now
    void get_rot_euler_rad(double pose_para[3]);
    void get_rot_quaternion(double pose_para[4]);
    void get_rot_matrix(Matrix3d& rot_mat);
    void set_rot_euler_rad(double yaw, double pitch, double roll);
    void set_rot_quaternion(double x, double y, double z, double w);
    void report_para();
};

Parameter::Parameter(stair_complete::offline_dyn_paraConfig &config) {
    stair_length = config.stair_length;
    stair_width = config.stair_width;
    stair_height = config.stair_height;
    stair_num = config.stair_num;
    mesh_resolution = config.mesh_resolution;

    RowVector3d offset_row;
    vec_tf_from_rotation(offset_row, config.offset_x, config.offset_y, config.offset_z,
                         config.init_yaw, config.init_pitch, config.init_roll);

    stair_init_pos[0] = config.init_x + offset_row(0, 0);
    stair_init_pos[1] = config.init_y + offset_row(0, 1);
    stair_init_pos[2] = config.init_z + offset_row(0, 2);
    truncation_distance = config.truncation_distance;

    if (config.point_generation==0)
        stair_mesh_base(stair_length, stair_width, stair_height, stair_num, mesh_resolution, mesh_base);
    else if (config.point_generation==1)
        stair_mesh_base_vertical_penalize(stair_length, stair_width, stair_height, stair_num, mesh_resolution, mesh_base);
    else if (config.point_generation==2)
    {
        stair_mesh_base_keep_edge(stair_length, stair_width, stair_height, stair_num, mesh_resolution, mesh_base);
    }


    double init_yaw, init_pitch, init_roll;
    init_yaw = config.init_yaw;
    init_pitch = config.init_pitch;
    init_roll = config.init_roll;
    quad.setEuler(
            angles::from_degrees(init_yaw),
            angles::from_degrees(init_pitch),
            angles::from_degrees(init_roll)
            );

}

Parameter::Parameter(int argc, char **argv) {

}

//TODO to enable it later
//Parameter::Parameter(void) {
//    ros::param::param("~/stair_length", stair_length, (float)1.164);     // length
//    ros::param::param("~/stair_width", stair_width, (float)0.244);    // width
//    ros::param::param("~/stair_height", stair_height, (float)0.126);    // height
//    ros::param::param("~/stair_num", stair_num, (float)2.0);       // number of stairs
//    ros::param::param("~/mesh_resolution", mesh_resolution, (float)0.05);
//
//    ros::param::param("~/init_x", this->stair_init_pos[0], (float)-2.2);
//    ros::param::param("~/init_y", this->stair_init_pos[1], (float)1.0);
//    ros::param::param("~/init_z", this->stair_init_pos[2], (float)0.0);
//
//    double init_yaw, init_pitch, init_roll;
//    ros::param::param("~/init_yaw", init_yaw, (float)0.0);
//    ros::param::param("~/init_pitch", init_pitch, (float)0.0);
//    ros::param::param("~/init_roll", init_roll, (float)-150.0);
//
//    quad.setEuler(
//            angles::from_degrees(init_yaw),
//            angles::from_degrees(init_pitch),
//            angles::from_degrees(init_roll));
//}

void Parameter::set_rot_euler_rad(double yaw, double pitch, double roll) {
    this->quad.setEuler(yaw, pitch, roll);
}

void Parameter::set_rot_quaternion(double x, double y, double z, double w) {
    this->quad.setValue(x, y, z, w);
}

void Parameter::get_rot_matrix(Matrix3d &rot_mat) {
    tf2::Matrix3x3 rot_matrix(quad);
    Matrix3d rot_mat_to_copy;
    rot_mat_to_copy <<  rot_matrix.getRow(0)[0], rot_matrix.getRow(0)[1], rot_matrix.getRow(0)[2],
                        rot_matrix.getRow(1)[0], rot_matrix.getRow(1)[1], rot_matrix.getRow(1)[2],
                        rot_matrix.getRow(2)[0], rot_matrix.getRow(2)[1], rot_matrix.getRow(2)[2];
    rot_mat = rot_mat_to_copy;
}

void Parameter::get_rot_quaternion(double pose_para[4]) {
    pose_para[0] = quad.getX();
    pose_para[1] = quad.getY();
    pose_para[2] = quad.getZ();
    pose_para[3] = quad.getW();
}

void Parameter::get_rot_euler_rad(double pose_para[3]) {
    tf2::Matrix3x3 rot_mat(quad);
    rot_mat.getEulerYPR(pose_para[0], pose_para[1], pose_para[2]);
}

void Parameter::report_para() {
    cout<<"stair init pose: "<<stair_init_pos[0] <<" "<< stair_init_pos[1]<<" "<< stair_init_pos[2]<<endl;
    cout<<"stair shape length, width, height "<<stair_length<<" "<< stair_width<<" "<<stair_height<<endl;
    cout<<"stair num and resolution "<<stair_num<<" "<<mesh_resolution<<endl;
    double quaternion[4];
    this->get_rot_quaternion(quaternion);
    cout<<"quaternion "<<quaternion[0]<<" "<<quaternion[1]<<" "<<quaternion[2]<<" "<<quaternion[3]<<" "<<endl;
    cout<<"truncation distance "<<truncation_distance<<endl;
    cout<<"reference of meshgrid"<<endl<<mesh_base<<endl;
}

#endif //STAIR_COMPLETE_PARAMETER_H
