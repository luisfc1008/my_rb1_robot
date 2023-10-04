#pragma once
#include "robot_info/hydraulic_system_monitor.h"
#include "robot_info/robot_info_class.h"
#include <ros/ros.h>

class AGVRobotInfo : public RobotInfo {

public:
  AGVRobotInfo();
  AGVRobotInfo(ros::NodeHandle *node_handle);

  HydraulicSystemMonitor sys_info;

  void get_oil_info();

protected:
  std::string oil_temp;
  std::string oil_level;
  std::string oil_pressure;

private:
  std::string maximum_payload = "maximum_payload: 100 Kg";
  void publish_data();
};