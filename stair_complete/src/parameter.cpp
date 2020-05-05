//
// Created by zhiyuan on 09.11.19.
//

#include "parameter.h"

// Check the description in corresponding .cpp file.

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


    // Different method to generate stair meshgrid
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
    stair_rot = angles::from_degrees(init_roll);

}

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

void Parameter::get_rot_euler_angle(double pose_para[3]) {
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

