//
// Created by zhiyuan on 13.11.19.
//

#ifndef STAIR_COMPLETE_UTIL_H
#define STAIR_COMPLETE_UTIL_H

#include <ros/ros.h>

#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox/io/layer_io.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace Eigen;

void tf2_quad_to_eigen_matrix(tf2::Quaternion quad, MatrixXd& rot_mat);
void grad_cal(MatrixXd &mesh, voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator,
        MatrixXd& grad, MatrixXd& reliability, bool normalize=false);

void grad_cal(MatrixXd &mesh, voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator,
        MatrixXd& grad, MatrixXd& reliability, bool normalize){

    int row_num = mesh.rows();

    grad.resize(row_num, 3);
    reliability.resize(row_num, 1);

    for (int i = 0; i < row_num ; ++i) {
        Eigen::Matrix<voxblox::FloatingPoint, 3, 1>  pos, gradient;
        bool rightness_gra, rightness_dis;
        float distance;
        pos<<mesh(i,0), mesh(i,1), mesh(i,2);
        rightness_gra = interpolator.getGradient(pos, &gradient, true);
        rightness_dis = interpolator.getDistance(pos, &distance, true);
        if (rightness_dis && rightness_gra)
            gradient = normalize ? gradient.normalized(): gradient.normalized() * distance;
        grad(i, 0) = gradient(0, 0);
        grad(i, 1) = gradient(1, 0);
        grad(i, 2) = gradient(2, 0);
        reliability(i, 0) = rightness_dis * rightness_gra;
    }
}

void tf2_quad_to_eigen_matrix(tf2::Quaternion quad, MatrixXd& rot_mat){
    tf2::Matrix3x3 tf_mat(quad);
    MatrixXd temp_mat(3, 3);
    temp_mat(0, 0) = tf_mat.getRow(0)[0];
    temp_mat(0, 1) = tf_mat.getRow(0)[1];
    temp_mat(0, 2) = tf_mat.getRow(0)[2];
    temp_mat(1, 0) = tf_mat.getRow(1)[0];
    temp_mat(1, 1) = tf_mat.getRow(1)[1];
    temp_mat(1, 2) = tf_mat.getRow(1)[2];
    temp_mat(2, 0) = tf_mat.getRow(2)[0];
    temp_mat(2, 1) = tf_mat.getRow(2)[1];
    temp_mat(2, 2) = tf_mat.getRow(2)[2];
    rot_mat = temp_mat;
}

#endif //STAIR_COMPLETE_UTIL_H
