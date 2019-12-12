# Install it
 1. Copy  files to catkin_ws
  2. cd to catkin_ws
  3. in terminal run catkin build voxblox_ros stair_detection



# Run the demo

in the terminal run the the three sentence:

1.  roslaunch stair_detection off_world_load_wooden.launch 
2. roslaunch stair_detection off_visualization.launch
3. roslaunch stair_detection off_optimizer.launch 

In the first launch file, the layer would be loaded. In the second launch file, the custom node would be loaded. 

Every time run the third launch file, the optimization process would be executed once. Then type enter in the second terminal, the optimized stair position would be updated.