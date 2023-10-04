#pragma once
#include "robot_info/robot_info_class.h"
#include <ros/ros.h>

class AGVRobotInfo : public RobotInfo {

public:
  AGVRobotInfo();
  AGVRobotInfo(ros::NodeHandle *node_handle);

private:
  std::string maximum_payload = "maximum_payload: 100 Kg";
  void publish_data();
};