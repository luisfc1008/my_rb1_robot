#include "robot_gui/robot_gui.h"
#include "ros/init.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui");
  
  
  RobotGUI rb1_gui("get_distance");
  rb1_gui.run();
  
  
  return 0;
}