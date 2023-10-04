#include "robot_info/robot_info_class.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/node_handle.h"
#include <ros/ros.h>

RobotInfo::RobotInfo(ros::NodeHandle *node_handle) {
  nh = node_handle;
  init_publisher_info();
}

void RobotInfo::init_publisher_info() {
  info_pub =
      nh->advertise<robotinfo_msgs::RobotInfo10Fields>("robot_info", 1000, 1);
  robotinfo_msgs::RobotInfo10Fields info_msg;
  ROS_INFO("ROS Publisher Created.");
  publish_data();
}

void RobotInfo::publish_data() {
  ROS_INFO("Void function called");
  info_msg.data_field_01 = robot_description;
  info_msg.data_field_02 = serial_number;
  info_msg.data_field_03 = ip_address;
  info_msg.data_field_04 = firmware_version;
  info_pub.publish(info_msg);
}
