# 1. Installation
 1. Copy  files to ROS_PATH/catkin_ws/src

 2. cd to catkin_ws/

 3. in terminal run 
    
```bash
      	$ catkin build voxblox_ros stair_complete
```

# 2. Run the demo

```bash
		$ roslaunch stair_complete offline_opt.launch 
```
In the panel click "*Enable Node*" to enable optimization of node.

Change the configuration, the optimization node in the back-end will automatically adjust the new configuration and optimize it.

<img src="./pic/dyn_reconfigure.png" alt="image-20200129185240638" style="zoom: 33%;" />



# 3. How to use the package 

To use the package, here is a "data analyze" demo. 

```bash
		$ roslaunch stair_complete data_analysis.launch 
```

It reads the different configuration saved in the corresponding .csv file,  traversals every configuration in the file, executes the optimization process,  and save the result in "result.csv" file.

There are two methods to 

1. Dynamic Reconfigure service 

   The feature, dynamic reconfigure, is actually also implemented a special ROSService. It is not difficult to directly use C++ API to call the service. Just write a client and then write as follows:

   ```C++
   stair_complete::offline_dyn_paraConfig config;
   client.setConfiguration(config);
   //C++
   ```
   ```Python
   client.update_configuration({"init_x":3.5, "init_y":1.2})
   #PYTHON
   ```
   For further more information about ros-package dynamic reconfigure, refer this [short tutorial](http://wiki.ros.org/dynamic_reconfigure/Tutorials). 

   

2. Code API

   Mainly two classes are used: *Stair_Complete*,  *Stair_Optimizer*.

   *Stair_Optimizer* uses Ceres-Solver to optimize the problem.

   *Stair_Complete* wraps *Stair_Optimizer* and features like loading tsdf layer, generating meshgrid.  

   ```C++
StairComplete st_complete;
   stair_complete::offline_dyn_paraConfig config;
   ```
   
   There are two typical cases to call the optimization. Firstly directly call the class method. The configuration of optimization setting is given by config. The optimization result is save in opt_record.
   
   ```C++
   Opt_Record opt_record;
   st_complete.stair_completion(config, opt_record);
   ```
   
   Another possibility is to manually call the dyn_callback function, which is a special wrap of *stair_completion* and is used for *dynamic_reconfigure* package.
   
   ```C++
   st_complete.dyn_callback(config, 0);
   Opt_Record my_record = st_complete.opt_record;
   ```
   
   In the optimization is mainly calling this part.
   
   ```C++
   //firstly generate meshgrid points, and load tsdf files.
   Stair_Interpolater4Ceres stair_interpolator4ceres;
   stair_interpolator4ceres.set_interpolator(&interpolator);
   StairOptimizer stair_optimizer(mesh_rotated, 
                               &stair_interpolator4ceres, st_init_pos);
   stair_optimizer.opt_epoc(500);
   ```
   
   The information is saved in *stair_optimizer.opt_record*.  

<img src="./pic/pic.png" alt="image-20200129185240638" style="zoom: 50%;" />


​    

# 4. Tutorial of single optimization process

Here is the tutorial to read the dynamic reconfigure optimization process. Mainly it includes main loop and the callback function.

```C++
int main(int argc,char **argv){
    ros::init(argc, argv, "offline_opt");
    dynamic_reconfigure::Server<stair_complete::offline_dyn_paraConfig> server;
    dynamic_reconfigure::Server<stair_complete::offline_dyn_paraConfig>::CallbackType f;
    StairComplete st_complete;
    f = boost::bind(&StairComplete::dyn_callback, &st_complete, _1, _2);
    server.setCallback(f);
    ROS_INFO("ok");
    ros::spin();
    return 0;
}

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

    if (voxblox::io::LoadLayer<voxblox::TsdfVoxel>(tsdf_path, &layer_ptr)) {
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

    //------report information after optimization-------------
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
```

## 	Set the dynamic_reconfigure server

Firstly set the server, the server is already implemented by package dynamic_reconfigure, we just set our optimization method as customized callback function. The callback function has to get two input, Config and level. Level generally is not used. Config is manaually defined in file ***config/offline_dyn_para.cfg*.**		

   ```C++
dynamic_reconfigure::Server<stair_complete::offline_dyn_paraConfig> server;
dynamic_reconfigure::Server<stair_complete::offline_dyn_paraConfig>::CallbackType f;
StairComplete st_complete;
f = boost::bind(&StairComplete::dyn_callback, &st_complete, _1, _2);
server.setCallback(f);
   ```

The config parameter has the attribute:

**stair_num**,  <font color="#dd0000">int</font>  

stair_number

**stair_length, stair_width, stair_height**,   <font color="#dd0000">dobule</font>  

stair size

**mesh_resolution**,  <font color="#dd0000">double</font>  

the distance of stair's meshgrid points

**init_x, init_y, init_z, init_pitch, init_yaw, init_roll**,  <font color="#dd0000">double</font>  

initial position and direction of stair

**offset_x, offset_y, offset_z**,  <font color="#dd0000">double</font> 

based on the initial x,y,z, additionally add movement "offset" in the rotated frame

**truncation_distance**,  <font color="#dd0000">double</font>  

truncation distance of tsdf 

**report_grad**,  <font color="#dd0000">bool</font>  

whether report the gradients information as text interminal

**do_optimization**,  <font color="#dd0000">bool</font>  

if not enabled, the service will only visualize the gradients and stairs in the given position

**m_opt_loop_n**,  <font color="#dd0000">int</font>  

maximal steps of ceres-solver

**point_generation**,  <font color="#dd0000">{0, 1, 2}</font>  

how to generate meshgrid, 0: normal version, 1: addtionally add one point in vertical plane, 2, always keep the edge points of stair plane,

**enable_node**,  <font color="#dd0000">bool</font>  

enable the service, otherwise do nothing by calling



## Load the tsdf layer

The tsdf layber path is given by rosparam, "/tsdf_save_path",  load the tsdf and publish TSDF mesh, finally we get interpolator, which is used in the optimization process.

```C++

voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_ptr;

std::string tsdf_path;
ros::param::get("/tsdf_save_path", tsdf_path);

if (voxblox::io::LoadLayer<voxblox::TsdfVoxel>(tsdf_path, &layer_ptr)) {
cout<<"loaded success"<<endl;
} else {
ROS_ERROR_STREAM("Failed to load tsdf from file.");
return false;
}
std::shared_ptr<voxblox::TsdfMap> tsdf = std::make_shared<voxblox::TsdfMap>(layer_ptr);
loadTsdf(tsdf, truncation_distance, publish_mesh);
voxblox::Interpolator<voxblox::TsdfVoxel> interpolator(tsdf->getTsdfLayerPtr());
```

## Optimization process

Firstly we initialize the classes, parameters as well as the stair's and gradient's visualizer. Parameter is initialized by config, but config is like ros message, based on the information, parameter will initialize the transform like grad to radian, euler angle to quaternion. Besides that, we will count the points that succeeds by corresponding gradients and distance interpolation.  

```C++
static Stair_Visualizer stair_vis("/stair_vis");
static Grad_Visualizer grad_vis("/grad_vis");
static Grad_Visualizer grad_vis_opted("/grad_vis_opted");
static Stair_Visualizer stair_vis_opted("/stair_vis_opted", 1.0);

Parameter para(config);
para.report_para();

MatrixXd mesh_rotated;
stair_mesh_base2rotation(para.mesh_base, mesh_rotated, para.quad);

double st_init_pos[3] = {para.stair_init_pos[0], para.stair_init_pos[1], para.stair_init_pos[2]};

double init_p_rate, final_p_rate;
init_p_rate = get_reliable_point_rate(mesh_rotated, para.stair_init_pos, interpolator);
```



as mentioned before, the optimization part is mainly called like:

```C++
Stair_Interpolater4Ceres stair_interpolator4ceres;
stair_interpolator4ceres.set_interpolator(&interpolator);
StairOptimizer stair_optimizer(mesh_rotated.cast <float> (), 
                               &stair_interpolator4ceres, st_init_pos);
stair_optimizer.opt_epoc(config.m_opt_loop_n);
```

The optimization was implemented in stair_optimizer.cpp,  to read the code, I highly recommend to firstly read this link  http://ceres-solver.org/interfacing_with_autodiff.html.  The code struct of this file is mostly identical with the example in the link
