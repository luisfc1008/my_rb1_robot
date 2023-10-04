#include "robot_info/agv_robot_info.h"
#include "robot_info/robot_info_class.h"
#include "ros/param.h"
#include <ros/ros.h>

AGVRobotInfo::AGVRobotInfo(ros::NodeHandle *node_handle) {
  nh = node_handle;
  get_oil_info();
  init_publisher_info();
}

void AGVRobotInfo::get_oil_info() {
  oil_temp = sys_info.get_oil_temp();
  oil_level = sys_info.get_oil_level();
  oil_pressure = sys_info.get_oil_pressure();
}

void AGVRobotInfo::publish_data() {
  ROS_INFO("Void function called");
  info_msg.data_field_01 = robot_description;
  info_msg.data_field_02 = serial_number;
  info_msg.data_field_03 = ip_address;
  info_msg.data_field_04 = firmware_version;
  info_msg.data_field_05 = maximum_payload;
  info_msg.data_field_06 = oil_temp;
  info_msg.data_field_07 = oil_level;
  info_msg.data_field_08 = oil_pressure;
  info_pub.publish(info_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "agv_robot_info_node");
  ros::NodeHandle nh;
  AGVRobotInfo ari = AGVRobotInfo(&nh);
  ros::spin();
}