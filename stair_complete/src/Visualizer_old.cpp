//
// Created by zhiyuan on 08.09.19.
//

#include <iostream>

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Polygon.h"


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <angles/angles.h>

#include "visualizer.h"

using namespace Eigen;  //Eigen has not found

using namespace std;


Stair_Visualization::Stair_Visualization(int num_in, double length_in, double width_in, double height_in) {
//    loop_click(20);
    markerArrayPub = nh.advertise<visualization_msgs::MarkerArray>("MarkerArray", 40);
    stair_info_sub = nh.subscribe("/stair_pose",10,&Stair_Visualization::visualizer_callback,this);
    stair_info_sub_euler = nh.subscribe("/stair_pose_euler",10,&Stair_Visualization::visualizer_callback_euler,this);
    stair_info_pub_euler = nh.advertise<geometry_msgs::PoseStamped>("/stair_pose", 40);

    height = height_in;
    width  = width_in;
    length = length_in;
    number = num_in;

    marker.header.frame_id="/odom";  //TODO to change as parameter
    marker2.header.frame_id="/odom";

    marker.ns = "basic_shapes";
    marker2.ns = "basic_shapes";

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker2.type = visualization_msgs::Marker::CUBE;
    marker2.action = visualization_msgs::Marker::ADD;

    marker.scale.x = length;
    marker.scale.y = 0.0001;
    marker.scale.z = height;

    marker2.scale.x = length;
    marker2.scale.y = width;
    marker2.scale.z = 0.0001;

    marker.color.a = 0.6;
    marker.color.r = 120;
    marker.color.g = 0;
    marker.color.b = 120;

    marker2.color.a = 0.6;
    marker2.color.r = 120;
    marker2.color.g = 120;
    marker2.color.b = 0;
}

void Stair_Visualization::visualizer_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    std::cout<<"visaualize call back"<<std::endl;
    std::cout<<"I heard and I publish position: "<<pose->pose.position.x<<" "<<pose->pose.position.y<<" "<<pose->pose.position.z<<std::endl;
    pub(pose);
}

void Stair_Visualization::reload_stair(void){
    double length_new, height_new, width_new;
    int st_num_new;

    ros::param::param("/stair_num", st_num_new, 2);
    ros::param::param("/stair_height", height_new, 0.126);
    ros::param::param("/stair_width", width_new, 0.244);
    ros::param::param("/stair_length", length_new, 1.164);

    this->length = length_new;
    this->width = width_new;
    this->height = height_new;
    this->number = st_num_new;

    marker.scale.x = length_new;
    marker.scale.y = 0.0001;
    marker.scale.z = height_new;

    marker2.scale.x = length_new;
    marker2.scale.y = width_new;
    marker2.scale.z = 0.0001;
}

void Stair_Visualization::pub(const geometry_msgs::PoseStamped::ConstPtr& global_pose){
    this->reload_stair();


    double global_x, global_y, global_z;
    global_x = global_pose->pose.position.x;
    global_y = global_pose->pose.position.y;
    global_z = global_pose->pose.position.z;

    double global_ori_x, global_ori_y, global_ori_z, global_ori_w;
    global_ori_x = global_pose->pose.orientation.x;
    global_ori_y = global_pose->pose.orientation.y;
    global_ori_z = global_pose->pose.orientation.z;
    global_ori_w = global_pose->pose.orientation.w;

    tf2::Quaternion quad(global_ori_x, global_ori_y, global_ori_z, global_ori_w);
    tf2::Matrix3x3 rot_matrx(quad);
    double matri_11, matri_12, matri_13, matri_21, matri_22, matri_23, matri_31, matri_32, matri_33;
    matri_11 = rot_matrx.getRow(0)[0];
    matri_12 = rot_matrx.getRow(0)[1];
    matri_13 = rot_matrx.getRow(0)[2];
    matri_21 = rot_matrx.getRow(1)[0];
    matri_22 = rot_matrx.getRow(1)[1];
    matri_23 = rot_matrx.getRow(1)[2];
    matri_31 = rot_matrx.getRow(2)[0];
    matri_32 = rot_matrx.getRow(2)[1];
    matri_33 = rot_matrx.getRow(2)[2];

    for(int num=0;num<number;num++){
        double temp_x, temp_y, temp_z;

        id_offset = (num-1)*2;
        // verticle stair

        marker.header.stamp = ros::Time::now();
        marker.id = id_offset;

        geometry_msgs::Pose pose;
        // eigen has some problem in using, and actually it should use tf tree would be more clear

        temp_x = matri_11 * marker.scale.x/2.0 + matri_12 * (num*width + marker.scale.y/2.0 ) + matri_13 * (num*height + marker.scale.z/2.0);
        temp_y = matri_21 * marker.scale.x/2.0 + matri_22 * (num*width + marker.scale.y/2.0 ) + matri_23 * (num*height + marker.scale.z/2.0);
        temp_z = matri_31 * marker.scale.x/2.0 + matri_32 * (num*width + marker.scale.y/2.0 ) + matri_33 * (num*height + marker.scale.z/2.0);

        pose.position.x = global_x + temp_x;
        pose.position.y = global_y + temp_y;
        pose.position.z = global_z + temp_z;
        marker.pose=pose;
        marker.pose.orientation.x = global_ori_x;
        marker.pose.orientation.y = global_ori_y;
        marker.pose.orientation.z = global_ori_z;
        marker.pose.orientation.w = global_ori_w;

        // horizontal stair

        marker2.header.stamp = ros::Time::now();
        marker2.id = id_offset+1;

        geometry_msgs::Pose pose2;

        temp_x = matri_11 * marker.scale.x/2.0 + matri_12 * (num*width + marker2.scale.y/2.0) + matri_13 * (num*height +height + marker2.scale.z/2.0 );
        temp_y = matri_21 * marker.scale.x/2.0 + matri_22 * (num*width + marker2.scale.y/2.0) + matri_23 * (num*height +height + marker2.scale.z/2.0 );
        temp_z = matri_31 * marker.scale.x/2.0 + matri_32 * (num*width + marker2.scale.y/2.0) + matri_33 * (num*height +height + marker2.scale.z/2.0 );
        pose2.position.x = global_x + temp_x;
        pose2.position.y = global_y + temp_y;
        pose2.position.z = global_z + temp_z;
        marker2.pose=pose2;
        marker2.pose.orientation.x = global_ori_x;
        marker2.pose.orientation.y = global_ori_y;
        marker2.pose.orientation.z = global_ori_z;
        marker2.pose.orientation.w = global_ori_w;

        markerArray.markers.push_back(marker);
        markerArray.markers.push_back(marker2);
    }
    markerArrayPub.publish(markerArray);
//    loop_click.sleep();
}

void Stair_Visualization::visualizer_callback_euler(const geometry_msgs::TwistStamped::ConstPtr& trans_euler)
{
    geometry_msgs::PoseStamped trans;
    tf2::Quaternion quad;
    quad.setEuler(angles::from_degrees(trans_euler->twist.angular.x),
                  angles::from_degrees(trans_euler->twist.angular.y),
                  angles::from_degrees(trans_euler->twist.angular.z));

    trans.header.frame_id = "/odom";
    trans.header.stamp = trans_euler->header.stamp;
    trans.pose.position.x = trans_euler->twist.linear.x;
    trans.pose.position.y = trans_euler->twist.linear.y;
    trans.pose.position.z = trans_euler->twist.linear.z;
    trans.pose.orientation.x = quad.getX();
    trans.pose.orientation.y = quad.getY();
    trans.pose.orientation.z = quad.getZ();
    trans.pose.orientation.w = quad.getW();

    stair_info_pub_euler.publish(trans);
}



Grad_Visualization::Grad_Visualization() {
    markerArrayPub = nh.advertise<visualization_msgs::MarkerArray>("GradArray", 40);
    stair_info_sub = nh.subscribe("/visualizer/grad_pose",10,&Grad_Visualization::visualizer_callback,this);

    marker.header.frame_id="/odom";  //TODO to change as parameter
    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD; // Attention here!

    marker.scale.x = 0.005;
    marker.scale.y = 0.04;
    marker.scale.z = 0.005;

    marker.color.a = 1;
    marker.color.r = 255;
    marker.color.g = 0;
    marker.color.b = 0;
}

void Grad_Visualization::pub(const geometry_msgs::PoseArray::ConstPtr& global_pose){
//    ros::Rate loop_click(20);
    visualization_msgs::MarkerArray markerArray;

    id_offset = 1;

    marker.header.stamp = ros::Time::now();

    for (unsigned long m_counter=0; m_counter < global_pose->poses.size(); m_counter++) {
        visualization_msgs::Marker marker_to_push = marker;
        geometry_msgs::Point point, point2;

        marker_to_push.id = id_offset + m_counter;
        point.x = global_pose->poses[m_counter].position.x;
        point.y = global_pose->poses[m_counter].position.y;
        point.z = global_pose->poses[m_counter].position.z;
        marker_to_push.points.push_back(point);
        point2.x = point.x - global_pose->poses[m_counter].orientation.x;
        point2.y = point.y - global_pose->poses[m_counter].orientation.y;
        point2.z = point.z - global_pose->poses[m_counter].orientation.z;
        marker_to_push.points.push_back(point2);
        if (global_pose->poses[m_counter].orientation.w == 0){
            marker_to_push.color.a = 1;
            marker_to_push.color.r = 88;
            marker_to_push.color.g = 88;
            marker_to_push.color.b = 88; //gray
        }
        markerArray.markers.push_back(marker_to_push);
    }


    markerArrayPub.publish(markerArray);
//    loop_click.sleep();
}

void Grad_Visualization::visualizer_callback(const geometry_msgs::PoseArray::ConstPtr& pose)
{
    std::cout<<"I publish arrow "<<std::endl;
    pub(pose);
}

Grad_Publisher::Grad_Publisher(void){
    chatter_pub = n.advertise<geometry_msgs::PoseArray>("/visualizer/grad_pose", 1000);
}

void Grad_Publisher::pointweise_pub_pseudo(MatrixXf &meshgrid) {
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = "odom";
//    poseArray.header.seq = 1;
    for (int i = 0; i < meshgrid.rows() ; ++i) {
        geometry_msgs::Pose pose;
        pose.position.x = meshgrid(i, 0);
        pose.position.y = meshgrid(i, 1);
        pose.position.z = meshgrid(i, 2);
        pose.orientation.x = 0.01;
        pose.orientation.y = 0.01;
        pose.orientation.z = 0.01;
        pose.orientation.w = 1;
        poseArray.poses.push_back(pose);
    }
    this->chatter_pub.publish(poseArray);
}

void Grad_Publisher::gradient_pub(MatrixXf &meshgrid){
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = "odom";

    for (int i = 0; i < meshgrid.rows() ; ++i) {

        geometry_msgs::Pose pose;
        pose.position.x = meshgrid(i, 0);
        pose.position.y = meshgrid(i, 1);
        pose.position.z = meshgrid(i, 2);
        pose.orientation.x = meshgrid(i, 3);
        pose.orientation.y = meshgrid(i, 4);
        pose.orientation.z = meshgrid(i, 5);
        pose.orientation.w = meshgrid(i, 6); //if reliable
        poseArray.poses.push_back(pose);
    }
    this->chatter_pub.publish(poseArray);
}

Grad_Publisher2::Grad_Publisher2(void){
    chatter_pub = n.advertise<geometry_msgs::PoseArray>("/grad_to_filter_marker", 1000);
}

void Grad_Publisher2::pointweise_pub_pseudo(MatrixXf &meshgrid) {
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = "odom";
//    poseArray.header.seq = 1;
    for (int i = 0; i < meshgrid.rows() ; ++i) {
        geometry_msgs::Pose pose;
        pose.position.x = meshgrid(i, 0);
        pose.position.y = meshgrid(i, 1);
        pose.position.z = meshgrid(i, 2);
        pose.orientation.x = 0.01;
        pose.orientation.y = 0.01;
        pose.orientation.z = 0.01;
        pose.orientation.w = 1;
        poseArray.poses.push_back(pose);
    }
    this->chatter_pub.publish(poseArray);
}

void Grad_Publisher2::gradient_pub(MatrixXf &meshgrid){
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = "odom";

    for (int i = 0; i < meshgrid.rows() ; ++i) {


        geometry_msgs::Pose pose;
        pose.position.x = meshgrid(i, 0);
        pose.position.y = meshgrid(i, 1);
        pose.position.z = meshgrid(i, 2);
        pose.orientation.x = meshgrid(i, 3);
        pose.orientation.y = meshgrid(i, 4);
        pose.orientation.z = meshgrid(i, 5);
        pose.orientation.w = meshgrid(i, 6); //if reliable
        poseArray.poses.push_back(pose);
    }
    this->chatter_pub.publish(poseArray);
}

void point_check(Grad_Publisher &grad_publisher, Parameters &parameters, MatrixXf &meshgrid_original){
    MatrixXf meshgrid = meshgrid_original;
    meshgrid_rotation(&parameters, meshgrid);
//    cout<<endl<<"roatated mesh"<<endl<<meshgrid<<endl;
    meshgrid_translation(&parameters, meshgrid);

    grad_publisher.pointweise_pub_pseudo(meshgrid);
    grad_publisher.pointweise_pub_pseudo(meshgrid);
    grad_publisher.pointweise_pub_pseudo(meshgrid);
    grad_publisher.pointweise_pub_pseudo(meshgrid);
    grad_publisher.pointweise_pub_pseudo(meshgrid);
    grad_publisher.pointweise_pub_pseudo(meshgrid);
    //TODO known bug, publish only once is not successful
};

void grad_check(Grad_Publisher &grad_publisher, Parameters &parameters, MatrixXf &meshgrid_original, voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator, bool mat_out){
    MatrixXf meshgrid = meshgrid_original;
    meshgrid_rotation(&parameters, meshgrid);
//    cout<<endl<<"roatated mesh"<<endl<<meshgrid<<endl;
    meshgrid_translation(&parameters, meshgrid);

    MatrixXf total_matrix(meshgrid.rows(), 9);
    for (int i = 0; i < meshgrid.rows() ; ++i) {
        Eigen::Matrix<voxblox::FloatingPoint, 3, 1>  pos, gradient;
        bool rightness_gra, rightness_dis;
        float distance;
        pos<<meshgrid(i,0), meshgrid(i,1), meshgrid(i,2);
        rightness_gra = interpolator.getGradient(pos, &gradient, true);
        rightness_dis = interpolator.getDistance(pos, &distance, true);
        if (rightness_dis && rightness_gra)
            gradient = gradient.normalized() * distance; //normalize gradient

        total_matrix(i, 0) = pos(0,0);
        total_matrix(i, 1) = pos(1,0);
        total_matrix(i, 2) = pos(2,0);
        total_matrix(i, 3) = gradient(0, 0);
        total_matrix(i, 4) = gradient(1, 0);
        total_matrix(i, 5) = gradient(2, 0);
        total_matrix(i, 6) = rightness_gra;
        total_matrix(i, 7) = distance;
        total_matrix(i, 8) = rightness_dis;
    }

    if (mat_out){
        cout<<endl<<"pose \t gradient \t if_success \t distance \t if_success"<<endl;
        cout<<total_matrix<<endl;
    }

    grad_publisher.gradient_pub(total_matrix);
    grad_publisher.gradient_pub(total_matrix);
    grad_publisher.gradient_pub(total_matrix);
    grad_publisher.gradient_pub(total_matrix);
}




void grad_filter_check(Grad_Publisher2 &grad_publisher, Grad_To_Filter_Publisher &grad_to_filter_publisher,
        Parameters &parameters, MatrixXf &meshgrid_original, voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator, bool mat_out){
    MatrixXf meshgrid = meshgrid_original;
    meshgrid_rotation(&parameters, meshgrid);

    meshgrid_translation(&parameters, meshgrid);

    MatrixXf total_matrix(meshgrid.rows(), 9);
    for (int i = 0; i < meshgrid.rows() ; ++i) {
        Eigen::Matrix<voxblox::FloatingPoint, 3, 1>  pos, gradient;
        bool rightness_gra, rightness_dis;
        float distance;
        pos<<meshgrid(i,0), meshgrid(i,1), meshgrid(i,2);
        rightness_gra = interpolator.getGradient(pos, &gradient, true);
        rightness_dis = interpolator.getDistance(pos, &distance, true);
        if (rightness_dis && rightness_gra)
            gradient = gradient.normalized() * distance; //normalize gradient

        total_matrix(i, 0) = pos(0,0);
        total_matrix(i, 1) = pos(1,0);
        total_matrix(i, 2) = pos(2,0);
        total_matrix(i, 3) = gradient(0, 0);
        total_matrix(i, 4) = gradient(1, 0);
        total_matrix(i, 5) = gradient(2, 0);
        total_matrix(i, 6) = rightness_gra;
        total_matrix(i, 7) = distance;
        total_matrix(i, 8) = rightness_dis;
    }

    if (mat_out){
        cout<<endl<<"pose \t gradient \t if_success \t distance \t if_success"<<endl;
        cout<<total_matrix<<endl;
    }

    grad_publisher.gradient_pub(total_matrix);

//    cout<<"test wait"<<endl;
//    geometry_msgs::PolygonConstPtr ptr_grad_filterd;
//
//    MatrixXf grads = total_matrix.block(0, 3, total_matrix.rows(), 3);
//    grad_to_filter_publisher.pub_grad(grads);

////    Wait for message doesn't work

//    cout<<"test wait"<<endl;
//    static ros::NodeHandle nh;
//
//    ros::Duration timeout(5);
//    ptr_grad_filterd = ros::topic::waitForMessage<geometry_msgs::Polygon>("grad_filtered", timeout);
//    cout<<"hahahahaha"<<endl;
//    if (ptr_grad_filterd != NULL){
//        cout<<"false"<<endl;
//    }
//    else{
//        cout<<"success"<<endl;
//    }
//    cout<<"the size is ....."<<ptr_grad_filterd->points.size()<<endl;
}

Stair_Publisher::Stair_Publisher(void) {
    chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/stair_pose", 1000);
}

//for degree
void Stair_Publisher::pub_pose_euler_radian(double x, double y, double z, double yaw, double pitch, double roll){
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "/odom";

    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;

    tf2::Quaternion quad;
    quad.setEuler(yaw, pitch, roll);

    msg.pose.orientation.x = quad.x();
    msg.pose.orientation.y = quad.y();
    msg.pose.orientation.z = quad.z();
    msg.pose.orientation.w = quad.w();

    chatter_pub.publish(msg);
    chatter_pub.publish(msg);
    chatter_pub.publish(msg);
}

Grad_Trajectory::Grad_Trajectory() {
    chatter_pub = n.advertise<visualization_msgs::MarkerArray>("/marker_grad_trajectory", 40);

    marker_template.header.frame_id="/odom";  //TODO to change as parameter
    marker_template.ns = "basic_shapes";
    marker_template.type = visualization_msgs::Marker::ARROW;
    marker_template.action = visualization_msgs::Marker::ADD; // Attention here!

    marker_template.scale.x = 0.02;
    marker_template.scale.y = 0.04;
    marker_template.scale.z = 0.005;

    marker_template.color.a = 1;
    marker_template.color.r = 255;
    marker_template.color.g = 255;
    marker_template.color.b = 255;

    marker_template.lifetime = ros::Duration();
}

void Grad_Trajectory::trajectory_reset() {
    markerArray.markers.clear();
    visualization_msgs::Marker marker = marker_template;
    marker.action = visualization_msgs::Marker::DELETEALL;
    markerArray.markers.push_back(marker);
}

void Grad_Trajectory::traj_pub_add(MatrixXf &pos, MatrixXf &grad_sacled){
    MatrixXf pos2 = pos - grad_sacled;  //Note here is minus
    int num_g = pos.rows();

    int id_offset = markerArray.markers.size(); //Note this sentence
    for (int m_counter=0; m_counter < num_g; m_counter++) {
        visualization_msgs::Marker marker_to_push = marker_template;
        marker_to_push.header.stamp = ros::Time::now();

        geometry_msgs::Point point, point2;

        marker_to_push.id = id_offset + m_counter;
        point.x = pos(m_counter, 0);
        point.y = pos(m_counter, 1);
        point.z = pos(m_counter, 2);
        marker_to_push.points.push_back(point);
        point2.x = pos2(m_counter, 0);
        point2.y = pos2(m_counter, 1);
        point2.z = pos2(m_counter, 2);
        marker_to_push.points.push_back(point2);
        markerArray.markers.push_back(marker_to_push);
    }
    chatter_pub.publish(markerArray);
}

Grad_To_Filter_Publisher::Grad_To_Filter_Publisher(void){
    chatter_pub = n.advertise<geometry_msgs::Polygon>("/grad_to_filter", 1000);
}

void Grad_To_Filter_Publisher::pub_grad(MatrixXf &grads){
    geometry_msgs::Polygon poly_to_filter;
    geometry_msgs::Point32 grad_single;

    int length = grads.rows();
    for (int i = 0; i < length; ++i) {
        grad_single.x = grads(i, 0);
        grad_single.y = grads(i, 1);
        grad_single.z = grads(i, 2);
        poly_to_filter.points.push_back(grad_single);
    }
    chatter_pub.publish(poly_to_filter);
}


