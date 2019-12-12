#include "handle_pointcloud.h"
#include <math.h>
#include "stair_optimizer_util.h"

using namespace Eigen;

//TODO namespace

void stair2meshgrid(float *stair_params, float step, MatrixXf& meshgrid ){
  float length, width, height;
  int stair_num;
  int num_point;

  length = stair_params[0];
  width  = stair_params[1];
  height = stair_params[2];
  stair_num  = (int) stair_params[3];
  float stair_num_float = stair_params[3];

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

      if ( fmod(meshgrid(i,1)  , width) < step) meshgrid(i,2)  += height;
    }
  }
  std::cout<<"meshgrid used "<<num_point<<" points"<<std::endl;
  return;
}


void meshgrid_rotation(Parameters *parameters, MatrixXf& meshgrid){
    tf2::Quaternion quad;
    quad.setEuler(parameters->stair_pos[3], parameters->stair_pos[4], parameters->stair_pos[5]);
    tf2::Matrix3x3 rot_matrx(quad);

    Matrix3f Rot_matrix;
    Rot_matrix<<rot_matrx.getRow(0)[0], rot_matrx.getRow(0)[1], rot_matrx.getRow(0)[2],
            rot_matrx.getRow(1)[0], rot_matrx.getRow(1)[1], rot_matrx.getRow(1)[2],
            rot_matrx.getRow(2)[0], rot_matrx.getRow(2)[1], rot_matrx.getRow(2)[2];
    meshgrid = meshgrid * (Rot_matrix.transpose());
//    cout<<"Rot"<<endl<<Rot_matrix<<endl<<endl;
    return;
}

void meshgrid_translation(Parameters *parameters, MatrixXf& meshgrid){
    RowVector3f translation;
    translation<< parameters->stair_pos[0], parameters->stair_pos[1], parameters->stair_pos[2];
    meshgrid.rowwise() += translation;
    return;
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


//TODO to merge with grad_check in Visulizer.cpp
//TODO to rewrite the input parameter
//TODO to think if normalize the gradient
void grad_cal(Parameters &parameters, MatrixXf &meshgrid_original, MatrixXf &total_info, voxblox::Interpolator<voxblox::TsdfVoxel> &interpolator){
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
    total_info = total_matrix;
}



void grad_summary(MatrixXf &total_info, MatrixXf &grad_summaried){
    int number = total_info.rows();
    MatrixXf grad = total_info.block(0, 3, number, 3 );
    VectorXf grad_belief = total_info.block(0, 6, number, 1 );
    VectorXf dis =  total_info.block(0, 7, number, 1 );
    VectorXf dis_belief = total_info.block(0, 8, number, 1 );

    //TODO and note recheck the logic here, removed the dis here since already multiplied in the gradient
//    VectorXf dis_discounted = dis.array() * dis_belief.array() * grad_belief.array();  //concern gradient rightness
    VectorXf dis_discounted = dis_belief.array() * grad_belief.array();
    MatrixXf grad_discounted = grad.array().colwise() * dis_discounted.array();

    grad_summaried = grad_discounted.colwise().sum();

//    cout<<"    ---------grad---summary------------"<<endl;
//    cout<<grad_summaried<<endl;
}




// Test function for meshgrid generation

//void opt_pseudo_test(){
//
//  std::cout<<"pseudo test"<<std::endl;
//
//  Parameters parameters;
//
//  parameters.stair_params[0] = 1.164;     // length
//  parameters.stair_params[1] = 0.244;     // width
//  parameters.stair_params[2] = 0.126;     // height
//  parameters.stair_params[3] = 2.0;       // number of stairs
//  parameters.step = 0.1;
//
//  Eigen::MatrixXf stair_array;
//
//  stair2meshgrid(parameters.stair_params, parameters.step, stair_array);
//
//  std::cout<<stair_array<<std::endl;
//}


