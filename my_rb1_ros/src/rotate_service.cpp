#include "ros/init.h"
#include "ros/ros.h"
#include "rotate_srv_msg/RotateServiceMessage.h"
#include <exception>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define _USE_MATH_DEFINES
#include <cmath>

class RotRB1 {

public:
  double current_ang;
  int rate_hz_ = 1;

  ros::Rate *rate_;

  // ROS Objects
  ros::NodeHandle nh_;

  // ROS Services
  ros::ServiceServer my_service;

  // ROS Subscribers
  ros::Subscriber deg_sub;

  // ROS Publishers
  ros::Publisher vel_pub;

  // ROS Messages
  geometry_msgs::Twist vel_msg;

  RotRB1() {
    my_service =
        nh_.advertiseService("/rotate_robot", &RotRB1::my_callback, this);
    // ROS_INFO("The Service /move_bb8_in_circle is READY");
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    deg_sub = nh_.subscribe("/odom", 1000, &RotRB1::counterCallBack, this);
    rate_ = new ros::Rate(rate_hz_);
  }

  void counterCallBack(const nav_msgs::Odometry::ConstPtr &msg) {
    float q_x = msg->pose.pose.orientation.x;
    float q_y = msg->pose.pose.orientation.y;
    float q_z = msg->pose.pose.orientation.z;
    float q_w = msg->pose.pose.orientation.w;

    // roll (x-axis rotation)
    // double sinr_cosp = 2 * (q_w * q_x + q_y * q_z);
    // double cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y);
    // double roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    // double sinp = std::sqrt(1 + 2 * (q_w * q_y - q_x * q_z));
    // double cosp = std::sqrt(1 - 2 * (q_w * q_y - q_x * q_z));
    // double pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q_w * q_z + q_x * q_y);
    double cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    current_ang = yaw * (180 / M_PI) + 180;

    // ROS_INFO_STREAM(current_ang);
  }

  void rotate_rb1() {
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.1;
    vel_pub.publish(vel_msg);
  }

  bool my_callback(rotate_srv_msg::RotateServiceMessage::Request &req,
                   rotate_srv_msg::RotateServiceMessage::Response &res) {
    ROS_INFO("Rotation service is called with %d degrees.", req.degrees);

    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);

    double turn_deg = req.degrees;
    float desired_angle = this->current_ang + turn_deg;
    float inital_ang = current_ang;
    float error = desired_angle - inital_ang;
    // ROS_INFO("error is: %f", error);

    if (req.degrees > 0.0) {
      ROS_INFO("Initalizing rotation by %d degrees ...", req.degrees); // left
      while (error > 0.1) {
        // ROS_INFO_STREAM(error);
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = -0.05;
        vel_pub.publish(vel_msg);
        // ROS_INFO("obtaining new error");
        inital_ang = current_ang;
        error = desired_angle - inital_ang;
        // ROS_INFO("new error obtained");
        ros::spinOnce();
        rate_->sleep();
      }

    } else {
      ROS_INFO("Initalizing rotation by %d degrees ...", req.degrees); // right
      while (error < -0.1) {
        // ROS_INFO_STREAM(error);
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.05;
        vel_pub.publish(vel_msg);
        // ROS_INFO("obtaining new error");
        inital_ang = current_ang;
        error = desired_angle - inital_ang;
        // ROS_INFO("new error obtained");
        ros::spinOnce();
        rate_->sleep();
      }
    }
    ROS_INFO("Rotation angle Reached!");
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);
    rate_->sleep();
    res.result = true;
    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_rb1");

  RotRB1 rotRB1;

  ros::spin();

  return 0;
};