//
// Created by zhiyuan on 29.01.20.
//
#include "stair_completion.h"

/*
 * Using this class for optimization process
 * It is possible to directly call "stair_completion" to do optimization,
 * otherwise is possible to use dyn_callback with dynamic reconfiguration,
 * Example -> stair_completion.cpp, offline_opt.launch
 */

/*
 * input: config, configuration of stair optimization
 * output: record of optimization, like stair position before and after optimization
 *         concretely see the class definition Opt_Record
 *
 * This function is main part of this package, this function do the optimization.
 * The pipeline: read tsdf layer -> generate stair mesh and transformation
 * -> call ceres and optimization -> report the information
 */

int StairComplete::stair_completion(stair_complete::offline_dyn_paraConfig &config, Opt_Record &opt_record){
    system("clear");
    ROS_INFO("call back begin");
    if (!config.enable_node ) return 0;

    //////--- read the vxblx layer and initialize interpolator--------------
    // Function Encapsulation is not implemented in this block
    // If do so, "Segementation error" occurs
    // It seems that the loded data from file is deleted after executing function
    float truncation_distance = config.truncation_distance;
    bool publish_mesh = true;
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_ptr;

    std::string tsdf_path;
    ros::param::get("/tsdf_save_path", tsdf_path);

    if (voxblox::io::LoadLayer<voxblox::TsdfVoxel>(
            tsdf_path,
            &layer_ptr)) {
        cout<<"loaded success"<<endl;
    } else {
        ROS_ERROR_STREAM("Failed to load tsdf from file.");
        return false;
    }
    std::shared_ptr<voxblox::TsdfMap> tsdf = std::make_shared<voxblox::TsdfMap>(layer_ptr);
    loadTsdf(tsdf, truncation_distance, publish_mesh);
    voxblox::Interpolator<voxblox::TsdfVoxel> interpolator(tsdf->getTsdfLayerPtr());
    ////---------------------------------------------------

    static Stair_Visualizer stair_vis("/stair_vis");
    static Grad_Visualizer grad_vis("/grad_vis");
    static Grad_Visualizer grad_vis_opted("/grad_vis_opted");
    static Stair_Visualizer stair_vis_opted("/stair_vis_opted", 1.0);
    // befor and after optimization, the stair visualization has different transparency

    // Note that the ROS-Publisher could not publish information immediately,
    // it publishes successfully after few seconds

    Parameter para(config);
    para.report_para();

    MatrixXd mesh_rotated;
    stair_mesh_base2rotation(para.mesh_base, mesh_rotated, para.quad);

    double st_init_pos[3] = {para.stair_init_pos[0], para.stair_init_pos[1], para.stair_init_pos[2]};

    double init_p_rate, final_p_rate;
    init_p_rate = get_reliable_point_rate(mesh_rotated, para.stair_init_pos, interpolator);

    //------report information before optimization-------------
    cout<<"before optimization"<<endl;
    stair_vis.pub_pose(config.stair_length, config.stair_width, config.stair_height,
                       config.stair_num, st_init_pos, para.quad);
    visualize_grad(para, interpolator, grad_vis, st_init_pos);
    if (config.report_grad)
        report_info(mesh_rotated, para.stair_init_pos, interpolator);
    // --------------------------------------------------

    //stop optimization if optimization is not used
    if (!config.do_optimization)
        return 0;

    //-------begin optimization ---------------
    Stair_Interpolater4Ceres stair_interpolator4ceres;
    stair_interpolator4ceres.set_interpolator(&interpolator);
    StairOptimizer stair_optimizer(mesh_rotated.cast <float> (), &stair_interpolator4ceres, st_init_pos);
    stair_optimizer.opt_epoc(config.m_opt_loop_n);

    //------report information after optimization-------------
    cout<<"after optimization"<<endl;
    visualize_grad(para, interpolator, grad_vis_opted, st_init_pos);
    stair_vis_opted.pub_pose(para.stair_length, para.stair_width, para.stair_height,
                             para.stair_num, st_init_pos, para.quad);
    if (config.report_grad)
        report_info(mesh_rotated, st_init_pos, interpolator);

    final_p_rate = get_reliable_point_rate(mesh_rotated, stair_optimizer.opt_variable, interpolator);

    opt_record = stair_optimizer.opt_record;

    opt_record.init_p_rate =init_p_rate;
    opt_record.final_p_rate=final_p_rate;

    ROS_INFO("call back ends");
    return 0;
};





