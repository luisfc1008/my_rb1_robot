#pragma once

#define CVUI_IMPLEMENTATION
#include "nav_msgs/Odometry.h"
#include "robot_gui/cvui.h"
#include "std_srvs/Trigger.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iomanip>
#include <sstream>
#include <robotinfo_msgs/RobotInfo10Fields.h>

class RobotGUI {
public:
  RobotGUI(const std::string &srv_name);
  void run();
  

private:
  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_msg;
  std::string twist_topic_name;
  float linear_velocity_step = 0.1;
  float angular_velocity_step = 0.1;

  ros::Subscriber sub_;
  nav_msgs::Odometry data;
  std::string topic_name;
  void msgCallback(const nav_msgs::Odometry::ConstPtr &msg);

  ros::Subscriber sub_info;
  robotinfo_msgs::RobotInfo10Fields info;
  void msgCallback2(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg2);
 
  ros::ServiceClient service_client;
  std_srvs::Trigger srv_req;
  std::string service_name;
  std::string last_service_call_msg;

  const std::string WINDOW_NAME = "CVUI ROS SIMPLE SUBSCRIBER";
};