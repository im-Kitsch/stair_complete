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

/*
 * This file is about to save the information of stair optimization
 * configuration, and it will generate the split of stair, i.e. meshgrid.
 * Additionally some rotation utility function is implemented.
 */

class Parameter{
public:
    double stair_init_pos[3]; // initial position x, y, z in meter
    double mesh_resolution; // the metric to split stair, unit in meter
    double stair_length, stair_width, stair_height;
    int stair_num;

    double truncation_distance;
    tf2::Quaternion quad; // stair direction, saved in quaternion
    MatrixXd mesh_base;

    Parameter(stair_complete::offline_dyn_paraConfig &config);

    // the next three function is used to get the information of stair rotation
    // respectively in euler angle, quaternion, rotation matrix
    void get_rot_euler_angle(double pose_para[3]);
    void get_rot_quaternion(double pose_para[4]);
    void get_rot_matrix(Matrix3d& rot_mat);

    //setting the rotation of stairs via euler angle or quaternion
    void set_rot_euler_rad(double yaw, double pitch, double roll); //unit in radian
    void set_rot_quaternion(double x, double y, double z, double w);

    // report the information of optimization configuration
    void report_para();
};

#endif //STAIR_COMPLETE_PARAMETER_H
