#ifndef _STAIR_OPTIMIZER_UTIL_H
#define _STAIR_OPTIMIZER_UTIL_H

using namespace Eigen;
#include "handle_pointcloud.h"
#include "stair_optimizer_util.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox/io/layer_io.h>

void stair2meshgrid(float *stair_params, float step, MatrixXf& meshgrid);
void meshgrid_rotation(Parameters *parameters, MatrixXf& meshgrid);
void meshgrid_translation(Parameters *parameters, MatrixXf& meshgrid);

void opt_pseudo_test();

double degree2radian(double deg);

bool loadTsdfFromFile(std::string tsdf_file_path, float truncation_distance, bool publish_mesh);
void loadTsdf(const std::shared_ptr<voxblox::TsdfMap> &tsdf, float truncation_distance, bool publish_mesh);
void publishMesh(ros::Publisher &pub, const std::shared_ptr<voxblox::MeshLayer> &mesh, std::string frame_id);
std::shared_ptr<voxblox::MeshLayer> computeMesh(const std::shared_ptr<voxblox::TsdfMap> &tsdf);

void grad_cal(Parameters &parameters, MatrixXf &meshgrid_original, MatrixXf &total_info, voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator);
void grad_summary(MatrixXf &total_info, MatrixXf &grad_summary);

#endif
