
#include <ros/ros.h>
#include "handle_pointcloud.h"
#include"stair_optimizer_util.h"
#include"stair_optimizer.h"

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"

using namespace Eigen;

using namespace std;

#include <tf2/LinearMath/Quaternion.h>

int main(int argc,char **argv)
{
    ros::init(argc,argv,"cut_pointcloud");

    Parameters parameters;
    // set parameters
    parameters.name = "/handled_pointcloud";
    parameters.node_path = "/voxblox_node/tsdf_map_out";

    ros::param::param("init_x", parameters.stair_pos[0], (float)-2.2);
    ros::param::param("init_y", parameters.stair_pos[1], (float)1.0);
    ros::param::param("init_z", parameters.stair_pos[2], (float)0.0);
    ros::param::param("init_yaw", parameters.stair_pos[3], (float)0.0);
    ros::param::param("init_pitch", parameters.stair_pos[4], (float)0.0);
    ros::param::param("init_roll", parameters.stair_pos[5], (float)-150.0);

    //degree and radian transformation TODO to improve
    parameters.stair_pos[3] = degree2radian(parameters.stair_pos[3]);
    parameters.stair_pos[4] = degree2radian(parameters.stair_pos[4]);
    parameters.stair_pos[5] = degree2radian(parameters.stair_pos[5]);

    ros::param::param("stair_length", parameters.stair_params[0], (float)1.164);     // length
    ros::param::param("stair_width", parameters.stair_params[1], (float)0.244);    // width
    ros::param::param("stair_height", parameters.stair_params[2], (float)0.126);    // height
    ros::param::param("stair_num", parameters.stair_params[3], (float)2.0);       // number of stairs

    ros::param::param("mesh_resolution", parameters.step, (float)0.05);
    parameters.edge_len = 0.05;
    parameters.argv = argv[0];

    //TODO verbose here
    tf2::Quaternion quad;
    quad.setEuler(parameters.stair_pos[3], parameters.stair_pos[4], parameters.stair_pos[5]);

    std::cout<<"initial position:"<<std::endl\
            <<"initial x y z: "<<parameters.stair_pos[0]<<" "\
                  <<parameters.stair_pos[1]<<" "<<parameters.stair_pos[2]<<std::endl\
            <<"initial yaw pitch roll: "<<parameters.stair_pos[3]<<" "<<parameters.stair_pos[4]\
                  <<" "<<parameters.stair_pos[5]<<std::endl\
            <<"(quaternion x y z w:"<<quad.x()<<" "<<quad.y()<<" "<<quad.z()<<" "<<quad.w()<<")"<<std::endl;
    std::cout<<"stairs setting: number lenght width height "<<parameters.stair_params[3]\
            <<" "<<parameters.stair_params[0]<<" "<<parameters.stair_params[1]<<" "\
             <<parameters.stair_params[2]<<std::endl;
    std::cout<<"meshgrid resolution: "<<parameters.step<<std::endl;


    handle_pointcloud handle_pointcloud1(parameters);

    //Important Bug! here doesn't publish
    geometry_msgs::PoseStamped pose; //TODO header info, time stamped, ID, frameID....
    pose.pose.position.x = parameters.stair_pos[0];
    pose.pose.position.y = parameters.stair_pos[1];
    pose.pose.position.z = parameters.stair_pos[2];
    pose.pose.orientation.w = quad.w();
    pose.pose.orientation.y = quad.y();
    pose.pose.orientation.z = quad.z();
    pose.pose.orientation.x = quad.x();
    handle_pointcloud1.pub.publish(pose);

    cout<<"I AM RUNNING !"<<endl;
    ros::spin();
}
