//
// Created by zhiyuan on 14.11.19.
//

#ifndef STAIR_COMPLETE_MESHGRID_UTIL_H
#define STAIR_COMPLETE_MESHGRID_UTIL_H

#include <tf2/LinearMath/Quaternion.h>

#include "Eigen/Core"

#include "util.h"

using namespace Eigen;

void stair_mesh_base(double length, double width, double height, int stair_num, double step, MatrixXd& meshgrid);
void stair_mesh_base2actual(MatrixXd& meshgrid_base, MatrixXd& meshgrid_actual, double pose[3], tf2::Quaternion quad);
void stair_mesh_base2rotation(MatrixXd& meshgrid_base, MatrixXd& meshgrid_actual, tf2::Quaternion quad);

void stair_mesh_base2actual(MatrixXd& meshgrid_base, MatrixXd& meshgrid_actual, double pose[3], tf2::Quaternion quad){
    MatrixXd rot_mat(3, 3);
    tf2_quad_to_eigen_matrix(quad, rot_mat);

    meshgrid_actual = meshgrid_base * rot_mat.transpose();

    RowVector3d translation;
    translation<<pose[0], pose[1], pose[2];
    meshgrid_actual.rowwise() += translation;
}

void stair_mesh_base2rotation(MatrixXd& meshgrid_base, MatrixXd& meshgrid_actual, tf2::Quaternion quad){
    MatrixXd rot_mat(3, 3);
    tf2_quad_to_eigen_matrix(quad, rot_mat);
    meshgrid_actual = meshgrid_base * rot_mat.transpose();
}

void stair_mesh_base(double length, double width, double height, int stair_num, double step, MatrixXd& meshgrid ){
    int num_point;

    float stair_num_float = stair_num;

    num_point = (int) ( floor(length/step+1) * floor(width * stair_num_float/step+1) );

    meshgrid.resize(num_point, 3);

    meshgrid(0,0) = 0.0;
    meshgrid(0,1) = 0.0;
    meshgrid(0,2) = 0.0 + height;

    for (int i =1; i<num_point; i++){
        meshgrid(i,0) = meshgrid(i-1,0) + step;
        meshgrid(i,1) = meshgrid(i-1,1) ;
        meshgrid(i,2) = meshgrid(i-1,2) ;
        if (meshgrid(i,0)  > length) {
            meshgrid(i,0)  = 0.0;
            meshgrid(i,1)  += step;

            if ( fmod(meshgrid(i,1)  , width) < step) {
                if ( fmod(meshgrid(i,1)  , width) > 0.)
                    meshgrid(i,2)  += height;
            }
        }
    }
}

void stair_mesh_base_vertical_penalize(double length, double width, double height, int stair_num, double step, MatrixXd& meshgrid ){
    int num_point;

    float stair_num_float = stair_num;

    num_point = (int) ( floor(length/step+1) * floor(width * stair_num_float/step+1) );

    meshgrid.resize(num_point + stair_num, 3);

    meshgrid(0,0) = 0.0;
    meshgrid(0,1) = 0.0;
    meshgrid(0,2) = 0.0 + height;

    for (int i =1; i<num_point; i++){
        meshgrid(i,0) = meshgrid(i-1,0) + step;
        meshgrid(i,1) = meshgrid(i-1,1) ;
        meshgrid(i,2) = meshgrid(i-1,2) ;
        if (meshgrid(i,0)  > length) {
            meshgrid(i,0)  = 0.0;
            meshgrid(i,1)  += step;

            if ( fmod(meshgrid(i,1)  , width) < step) {
                if ( fmod(meshgrid(i,1)  , width) > 0.)
                    meshgrid(i,2)  += height;
            }
        }
    }

    for (int j = 0; j < stair_num ; ++j) {
        meshgrid(j+num_point, 0) = length/2;
        meshgrid(j+num_point, 1) = width * (double) j;
        meshgrid(j+num_point, 2) = ((double)j+0.5) * height;
    }
}

void stair_mesh_base_keep_edge(double length, double width, double height, int stair_num, double step, MatrixXd& meshgrid ){
    int num_point, num_point_per_plane;

    int num_length, num_width;
    num_length = (floor(length/step) + 1);
    num_width = (floor(width/step)+1);

    num_point_per_plane =  num_length * num_width;
    num_point = num_point_per_plane * stair_num;

    meshgrid.resize(num_point, 3);

    for (int k = 0; k < stair_num; ++k) {
        for (int i = 0; i < num_length ; ++i) {
            for (int j = 0; j < num_width; ++j) {
                meshgrid(k*num_point_per_plane+i*num_width+j, 0) = step * (double)i;
                meshgrid(k*num_point_per_plane+i*num_width+j, 1) = step * (double)j + (double)k*width;
                meshgrid(k*num_point_per_plane+i*num_width+j, 2) = height + (double)k * height;
            }
        }
    }
}

#endif //STAIR_COMPLETE_MESHGRID_UTIL_H
