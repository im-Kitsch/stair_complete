#include <iostream>

#include <ros/ros.h>


#include "visualizer.h"

using namespace std;


int main(int argc, char **argv){
  ros::init(argc, argv, "show_markers");

  double length, height, width;
  int st_num;

  ros::param::param("/stair_num", st_num, 2);
  ros::param::param("/stair_height", height, 0.126);
  ros::param::param("/stair_width", width, 0.244);
  ros::param::param("/stair_length", length, 1.164);
  std::cout<<"intialization of setting "<<std::endl;
  std::cout<<"number: "<<st_num<<" length: "<<length<<" width: "<<width<<" height: "<<height<<std::endl;


  Stair_Visualization stair_visualization(st_num, length, width, height);
  Grad_Visualization grad_visualization;

  ros::spin();
  return 0;
}
