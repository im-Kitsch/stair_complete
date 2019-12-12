#include <iostream>
#include <typeinfo>
#include <ros/ros.h>

#include <voxblox/core/common.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox/interpolator/interpolator.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/conversions_inl.h>

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using namespace Eigen;

#include "stair_optimizer.h"
#include "stair_optimizer_util.h"

#include "handle_pointcloud.h"

#include "geometry_msgs/PoseStamped.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>



using namespace std;

handle_pointcloud::handle_pointcloud(Parameters parameters){
    this->params = parameters;

    pub = nh.advertise<geometry_msgs::PoseStamped>("/stair_pose", 50);
    sub = nh.subscribe(params.node_path,10,&handle_pointcloud::call_back,this);

    tf2::Quaternion quad;
    quad.setEuler(parameters.stair_pos[3], parameters.stair_pos[4], parameters.stair_pos[5]);
}
//const Layer<VoxelType>* layer
void handle_pointcloud::call_back(const voxblox_msgs::Layer& input){
    cout<<"I RECEIVED INPUT !"<<endl;

    // convert the voxblox layer msg to the internal layer type
    std::shared_ptr<voxblox::TsdfMap> tsdf;
    voxblox::TsdfMap::Config tsdf_config;
    tsdf_config.tsdf_voxel_size = static_cast<voxblox::FloatingPoint>(input.voxel_size);
    tsdf.reset(new voxblox::TsdfMap(tsdf_config));
    voxblox::deserializeMsgToLayer<voxblox::TsdfVoxel>(input, tsdf->getTsdfLayerPtr());
    voxblox::Interpolator<voxblox::TsdfVoxel> interpolator(tsdf->getTsdfLayerPtr());

    // optimization and get the position of the stairs

    Stair_Interpolater4Ceres stair_interpolator4ceres;
    stair_interpolator4ceres.set_interpolator(&interpolator);

    double stair_pos[3];
    stair_pos[0] = params.stair_pos[0];
    stair_pos[1] = params.stair_pos[1];
    stair_pos[2] = params.stair_pos[2];

    MatrixXf meshgrid;
    float step = params.step;
    stair2meshgrid(this->params.stair_params,  step,  meshgrid );

    // ------------------TODO to improve ---------------
    tf2::Quaternion quad;
    quad.setEuler(params.stair_pos[3], params.stair_pos[4], params.stair_pos[5]);
    tf2::Matrix3x3 rot_matrx(quad);

    Matrix3f Rot_matrix;
    Rot_matrix<<rot_matrx.getRow(0)[0], rot_matrx.getRow(0)[1], rot_matrx.getRow(0)[2],
                rot_matrx.getRow(1)[0], rot_matrx.getRow(1)[1], rot_matrx.getRow(1)[2],
                rot_matrx.getRow(2)[0], rot_matrx.getRow(1)[1], rot_matrx.getRow(2)[2];
    meshgrid = meshgrid * Rot_matrix.transpose();

//    cout<<meshgrid;

    int mesh_counter = meshgrid.rows();
    MatrixXf GradientMatrix(mesh_counter, 3);
    MatrixXf DistanceMatrix(mesh_counter, 1);
    MatrixXf AccuGra(1, 3);
    AccuGra<<0, 0, 0;

    cout<<"x \t y \t z \t dis \t gradient"<<endl;
    for (int ii=0;ii<mesh_counter;ii++) {
      voxblox::FloatingPoint dis;
      voxblox::Point query_p, grad;
      bool grad_if_true;
      bool dis_if_true;
      query_p(0, 0) = meshgrid(ii, 0) + stair_pos[0];
      query_p(1, 0) = meshgrid(ii, 1) + stair_pos[1];
      query_p(2, 0) = meshgrid(ii, 2) + stair_pos[2];
      dis_if_true = interpolator.getDistance(query_p, &dis, true);
      grad_if_true = interpolator.getGradient(query_p, &grad, true);

      AccuGra(0, 0) += dis * grad(0, 0);
      AccuGra(0, 1) += dis * grad(1, 0);
      AccuGra(0, 2) += dis * grad(2, 0);

//      DistanceMatrix(ii) = dis;
//      GradientMatrix(ii, 0) = grad(0, 0);
//      GradientMatrix(ii, 1) = grad(1, 0);
//      GradientMatrix(ii, 2) = grad(2, 0);

      cout<<"dis/grad_ifTrue: "<<dis_if_true<<" "<<grad_if_true<<" "<<query_p(0,0)<<"\t"<<query_p(1,0)<<"\t"<<query_p(2,0)\
         <<"\t"<<dis<<"\t"<<grad(0, 0)<<"\t"<<grad(1,0)<<"\t"<<grad(2,0)<<endl;
    }
    cout<<"first step opt, "<<AccuGra(0, 0)<<" "<<AccuGra(0,1)<<" "<<AccuGra(0,2)<<" "<<endl;


    // ----------------to change --------------------

    StairOptimizer stair_optimizer(meshgrid, &stair_interpolator4ceres, stair_pos);

    geometry_msgs::PoseStamped pose; //TODO header info, time stamped, ID, frameID....
    pose.pose.position.x = stair_optimizer.opt_variable[0];
    pose.pose.position.y = stair_optimizer.opt_variable[1];
    pose.pose.position.z = stair_optimizer.opt_variable[2];
    pose.pose.orientation.w = quad.w();
    pose.pose.orientation.y = quad.y();
    pose.pose.orientation.z = quad.z();
    pose.pose.orientation.x = quad.x();
    this->pub.publish(pose);

    stair_optimizer.opt_epoc();



    // publicate the stair information

    // visualization of the stair model

    cout<<"Opt finished type enter to continue !"<<endl;
    std::cin.get();
    std::cout<<"okay continue!"<<std::endl;

    pose.pose.position.x = stair_optimizer.opt_variable[0];
    pose.pose.position.y = stair_optimizer.opt_variable[1];
    pose.pose.position.z = stair_optimizer.opt_variable[2];
    this->pub.publish(pose);
}
