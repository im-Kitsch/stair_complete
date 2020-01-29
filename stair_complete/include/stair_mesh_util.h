//
// Created by zhiyuan on 14.11.19.
//

#ifndef STAIR_COMPLETE_MESHGRID_UTIL_H
#define STAIR_COMPLETE_MESHGRID_UTIL_H

#include <tf2/LinearMath/Quaternion.h>

#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox/io/layer_io.h>

#include "Eigen/Core"



using namespace Eigen;

// utility functions, like meshgrid generation, report grad, translation


void stair_mesh_base2actual(MatrixXd& meshgrid_base, MatrixXd& meshgrid_actual, double pose[3], tf2::Quaternion quad);
void stair_mesh_base2rotation(MatrixXd& meshgrid_base, MatrixXd& meshgrid_actual, tf2::Quaternion quad);

void stair_mesh_base(double length, double width, double height, int stair_num, double step, MatrixXd& meshgrid);
void stair_mesh_base_vertical_penalize(double length, double width, double height, int stair_num, double step, MatrixXd& meshgrid );
void stair_mesh_base_keep_edge(double length, double width, double height, int stair_num, double step, MatrixXd& meshgrid );

void report_info(MatrixXd& meshgrid_rotated, double pose[3], voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator);

void vec_tf_from_rotation(RowVector3d& offset_vec, double x, double y, double z,
                          double yaw, double pitch, double roll);

double get_reliable_point_rate(MatrixXd& meshgrid_rotated, double pose[3],
                               voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator);

void tf2_quad_to_eigen_matrix(tf2::Quaternion quad, MatrixXd& rot_mat);
#endif //STAIR_COMPLETE_MESHGRID_UTIL_H
