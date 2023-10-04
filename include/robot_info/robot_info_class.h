#pragma once
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <ros/ros.h>

class RobotInfo {

public:
  RobotInfo(){};
  RobotInfo(ros::NodeHandle *node_handle);
  std::string info_topic = "robot_info";
  ros::Publisher info_pub;
  robotinfo_msgs::RobotInfo10Fields info_msg;

protected:
  ros::NodeHandle *nh;
  void init_publisher_info();
  virtual void publish_data();
  std::string robot_description = "robot_description: Mir100yummy";
  std::string serial_number = "serial_number: 567A359";
  std::string ip_address = "ip_address: 169.254.5.180";
  std::string firmware_version = "firmware_version: 3.5.8";

};