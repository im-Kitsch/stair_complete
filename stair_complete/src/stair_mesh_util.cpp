#include <tf2/LinearMath/Quaternion.h>

#include "Eigen/Core"

#include <ros/ros.h>

#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox/io/layer_io.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


#include "stair_mesh_util.h"

using namespace Eigen;

#include <angles/angles.h>

/*
 * this function report the information about the gradients of all meshgrid points
 * input: meshgrid_rotated, the meshgrid points with direction
 *        pose, stair position
 *        interpolator
 */
void report_info(MatrixXd& meshgrid_rotated, double pose[3], voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator)
{
    int num_rows = meshgrid_rotated.rows();
    RowVector3d trans_vec;
    trans_vec<<pose[0], pose[1], pose[2];
    MatrixXd meshgrid = meshgrid_rotated;
    meshgrid.rowwise() += trans_vec;

    MatrixXd total_info(num_rows, 8);
    total_info.block(0, 0, num_rows, 3) = meshgrid;

    for (int i = 0; i < num_rows ; ++i) {
        Eigen::Matrix<voxblox::FloatingPoint, 3, 1>  pos, gradient;
        bool rightness_gra, rightness_dis;
        float distance;
        pos<<meshgrid(i,0), meshgrid(i,1), meshgrid(i,2);
        rightness_gra = interpolator.getGradient(pos, &gradient, true);
        rightness_dis = interpolator.getDistance(pos, &distance, true);

        total_info(i, 3) = gradient(0, 0);
        total_info(i, 4) = gradient(1, 0);
        total_info(i, 5) = gradient(2, 0);
        total_info(i, 6) = distance;
        total_info(i, 7) = rightness_dis * rightness_gra;
    }
    std::cout<<"pose    gradient    distance   reliability"<<std::endl;
    std::cout<<total_info<<std::endl;
    double reliable_point_percent;
    reliable_point_percent = (double) total_info.col(7).sum() / (double) total_info.rows();
    std::cout<<"reliable points percent  "<<reliable_point_percent<<std::endl;
}

/*
 * inverse change
 * given coordinates vec_1 in the rotated frame,
 * return the coordinates vec_0 in the frame before rotation
 * input: double x, y, z, vec_1
 *        double yaw pitch roll, rotation angle
 * output: RowVector3d offset_vec
 */
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

// count valid points and calculate the percentage of valid points of meshgrid points.
double get_reliable_point_rate(MatrixXd& meshgrid_rotated, double pose[3],
                               voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator)
{
    int num_rows = meshgrid_rotated.rows();
    RowVector3d trans_vec;
    trans_vec<<pose[0], pose[1], pose[2];
    MatrixXd meshgrid = meshgrid_rotated;
    meshgrid.rowwise() += trans_vec;

    MatrixXd total_info(num_rows, 8);
    total_info.block(0, 0, num_rows, 3) = meshgrid;

    for (int i = 0; i < num_rows ; ++i) {
        Eigen::Matrix<voxblox::FloatingPoint, 3, 1>  pos, gradient;
        bool rightness_gra, rightness_dis;
        float distance;
        pos<<meshgrid(i,0), meshgrid(i,1), meshgrid(i,2);
        rightness_gra = interpolator.getGradient(pos, &gradient, true);
        rightness_dis = interpolator.getDistance(pos, &distance, true);

        total_info(i, 3) = gradient(0, 0);
        total_info(i, 4) = gradient(1, 0);
        total_info(i, 5) = gradient(2, 0);
        total_info(i, 6) = distance;
        total_info(i, 7) = rightness_dis * rightness_gra;
    }

    double reliable_point_percent;
    reliable_point_percent = (double) total_info.col(7).sum() / (double) total_info.rows();

    return reliable_point_percent;
}


// transformation of meshgrid, with rotation and translation
void stair_mesh_base2actual(MatrixXd& meshgrid_base, MatrixXd& meshgrid_actual, double pose[3], tf2::Quaternion quad){
    MatrixXd rot_mat(3, 3);
    tf2_quad_to_eigen_matrix(quad, rot_mat);

    meshgrid_actual = meshgrid_base * rot_mat.transpose();

    RowVector3d translation;
    translation<<pose[0], pose[1], pose[2];
    meshgrid_actual.rowwise() += translation;
}

// transformation of meshgrid, only rotation
void stair_mesh_base2rotation(MatrixXd& meshgrid_base, MatrixXd& meshgrid_actual, tf2::Quaternion quad){
    MatrixXd rot_mat(3, 3);
    tf2_quad_to_eigen_matrix(quad, rot_mat);
    meshgrid_actual = meshgrid_base * rot_mat.transpose();
}

// normal method for generating meshgrid points
// for compareness, look the paper, ch6.2 Influence of sampling point methods
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

            // TODO a bug is here, e.g. width = 0.24 and mesh resolution is 0.12
            if ( fmod(meshgrid(i,1)  , width) < step) {
                if ( fmod(meshgrid(i,1)  , width) > 0.)
                    meshgrid(i,2)  += height;
            }
        }
    }
}

// based on normal method for generating meshgrid points,
// addtionally add a point for every vertical plane
// for compareness, look the paper, ch6.2 Influence of sampling point methods
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

// based on normal method for generating meshgrid points,
// But keep that the edge points are always generated
// for compareness, look the paper, ch6.2 Influence of sampling point methods
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

//transform from quaternion to rotation matrix
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
