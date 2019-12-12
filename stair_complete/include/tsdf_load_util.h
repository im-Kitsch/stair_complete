//
// Created by zhiyuan on 14.11.19.
//

#ifndef STAIR_COMPLETE_TSDF_LOAD_UTIL_H
#define STAIR_COMPLETE_TSDF_LOAD_UTIL_H

#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox/io/layer_io.h>

void loadTsdf(const std::shared_ptr<voxblox::TsdfMap> &tsdf, float truncation_distance, bool publish_mesh);
bool loadTsdfFromFile(std::string tsdf_file_path, float truncation_distance, bool publish_mesh);
void publishMesh(ros::Publisher &pub, const std::shared_ptr<voxblox::MeshLayer> &mesh, std::string frame_id);
std::shared_ptr<voxblox::MeshLayer> computeMesh(const std::shared_ptr<voxblox::TsdfMap> &tsdf);

void loadTsdf(const std::shared_ptr<voxblox::TsdfMap> &tsdf, float truncation_distance, bool publish_mesh)
{
//    sdf_wrapper_.setTsdfMap(tsdf, truncation_distance);  //Interpolated TSDF?

//    publishTsdfSlice(tsdf_slice_pub_, tsdf);  //seems like publishing pcl?
    if (publish_mesh) {
        ros::NodeHandle nh_;
        ros::Publisher mesh_pub_ = nh_.advertise<voxblox_msgs::Mesh>("mesh", 10, true);
        std::shared_ptr<voxblox::MeshLayer> mesh = computeMesh(tsdf);

        publishMesh(mesh_pub_, mesh, "/odom");
    }
}

//TODO faild to return interpolater
bool loadTsdfFromFile(std::string tsdf_file_path, float truncation_distance, bool publish_mesh)
{
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_ptr;
    if (voxblox::io::LoadLayer<voxblox::TsdfVoxel>(tsdf_file_path, &layer_ptr)) {
        std::shared_ptr<voxblox::TsdfMap>  tsdf = std::make_shared<voxblox::TsdfMap>(layer_ptr);
        loadTsdf(tsdf, truncation_distance, publish_mesh);
    } else {
        ROS_ERROR_STREAM("Failed to load tsdf from file.");
        return false;
    }
    return true;
}

std::shared_ptr<voxblox::MeshLayer> computeMesh(const std::shared_ptr<voxblox::TsdfMap> &tsdf)
{
    // Publish mesh
    voxblox::MeshIntegratorConfig mesh_config;
    mesh_config.min_weight = 0.1f;
    auto mesh_layer = std::make_shared<voxblox::MeshLayer>(tsdf->block_size());
    auto mesh_integrator = std::make_shared<voxblox::MeshIntegrator<voxblox::TsdfVoxel>>(mesh_config, tsdf->getTsdfLayerPtr(), mesh_layer.get());

    bool only_mesh_updated_blocks = false;
    bool clear_updated_flag = true;
    mesh_integrator->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
    return mesh_layer;
}

void publishMesh(ros::Publisher &pub, const std::shared_ptr<voxblox::MeshLayer> &mesh, std::string frame_id)
{
    voxblox_msgs::Mesh mesh_msg;
    voxblox::generateVoxbloxMeshMsg(mesh, voxblox::ColorMode::kNormals, &mesh_msg);
    mesh_msg.header.frame_id = frame_id;
    pub.publish(mesh_msg);
}

#endif //STAIR_COMPLETE_TSDF_LOAD_UTIL_H
