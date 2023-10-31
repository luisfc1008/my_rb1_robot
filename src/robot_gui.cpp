#include "robot_gui/robot_gui.h"
#include "ros/init.h"
#include "ros/spinner.h"

RobotGUI::RobotGUI(const std::string &srv_name) {
  // Initialize ROS node
  ros::NodeHandle nh;
  sub_info = nh.subscribe("robot_info", 10, &RobotGUI::msgCallback2, this);
  sub_ = nh.subscribe("odom", 10, &RobotGUI::msgCallback, this);
  twist_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  service_client = nh.serviceClient<std_srvs::Trigger>(srv_name);
  service_name = srv_name;
}

void RobotGUI::msgCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  data = *msg;
}

void RobotGUI::msgCallback2(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg2) {
  info = *msg2;
}

void RobotGUI::run() {
  int frame_x = 300;
  int frame_y = 530;
  int but_size = 70;
  int control_refx = (frame_x / 2) - (but_size / 2);
  int control_refy = (frame_y / 2) - (but_size / 2);

  cv::Mat frame = cv::Mat(600, 300, CV_8UC3);

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    ros::Rate rate(10);
    // Fill the frame with a nice color
    frame = cv::Scalar(148, 117, 64);

    // Create window at (40, 20) with size 250x80 (width x height) and title
    cvui::window(frame, control_refx - 75, 10, 220, 140, "Topic: robot_info");

    // Show how many times the button has been clicked inside the window.
    cvui::printf(frame, control_refx - 70, 32, 0.4, 0xFFFFFF, "%s",
                 info.data_field_01.c_str());
    cvui::printf(frame, control_refx - 70, 32 + (1 * 12), 0.4, 0xFFFFFF, "%s",
                 info.data_field_02.c_str());
    cvui::printf(frame, control_refx - 70, 32 + (2 * 12), 0.4, 0xFFFFFF, "%s",
                 info.data_field_03.c_str());
    cvui::printf(frame, control_refx - 70, 32 + (3 * 12), 0.4, 0xFFFFFF, "%s",
                 info.data_field_04.c_str());
    cvui::printf(frame, control_refx - 70, 32 + (4 * 12), 0.4, 0xFFFFFF, "%s",
                 info.data_field_05.c_str());
    cvui::printf(frame, control_refx - 70, 32 + (5 * 12), 0.4, 0xFFFFFF, "%s",
                 info.data_field_06.c_str());
    cvui::printf(frame, control_refx - 70, 32 + (6 * 12), 0.4, 0xFFFFFF, "%s",
                 info.data_field_07.c_str());
    cvui::printf(frame, control_refx - 70, 32 + (7 * 12), 0.4, 0xFFFFFF, "%s",
                 info.data_field_08.c_str());
    cvui::printf(frame, control_refx - 70, 32 + (8 * 12), 0.4, 0xFFFFFF, "%s",
                 info.data_field_09.c_str());
    cvui::printf(frame, control_refx - 70, 32 + (9 * 12), 0.4, 0xFFFFFF, "%s",
                 info.data_field_10.c_str());

    //-------------------------------------------------------------------//
    // Show a button at position x = 40, y = 80
    if (cvui::button(frame, control_refx, control_refy, but_size, but_size,
                     "Stop")) {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
    }

    if (cvui::button(frame, control_refx + 75, control_refy, but_size, but_size,
                     "Right")) {
      twist_msg.angular.z = twist_msg.angular.z - angular_velocity_step;
    }

    if (cvui::button(frame, control_refx - 75, control_refy, but_size, but_size,
                     "Left")) {
      twist_msg.angular.z = twist_msg.angular.z + angular_velocity_step;
    }

    if (cvui::button(frame, control_refx, control_refy - 75, but_size, but_size,
                     "Forward")) {
      twist_msg.linear.x = twist_msg.linear.x + linear_velocity_step;
    }

    if (cvui::button(frame, control_refx, control_refy + 75, but_size, but_size,
                     "Backward")) {
      twist_msg.linear.x = twist_msg.linear.x - linear_velocity_step;
    }

    twist_pub_.publish(twist_msg);

    //--------------------------------------------------------------------//

    int controls_y = control_refy + 75 + 70;

    // Create window at (320, 20) with size 120x40 (width x height) and title
    cvui::window(frame, control_refx - 75, controls_y + 5, 105, 40,
                 "Linear vel:");
    // Show the current velocity inside the window
    cvui::printf(frame, control_refx - 75 + 10, controls_y + 30, 0.35, 0xff0000,
                 "%.02f m/sec", twist_msg.linear.x);

    // Create window at (320, 20) with size 120x40 (width x height) and title
    cvui::window(frame, control_refx - 75 + 115, controls_y + 5, 105, 40,
                 "Angular vel:");
    // Show the current velocity inside the window
    cvui::printf(frame, control_refx + 38 + 10, controls_y + 30, 0.35, 0xff0000,
                 "%.02f rad/sec", twist_msg.angular.z);
    //-------------------------------------------------------------------------//

    // Create window at (40, 20) with size 250x80 (width x height) and title
    cvui::window(frame, control_refx - 75, controls_y + 55, 70, 70, "X");
    // Show how many times the button has been clicked inside the window.
    cvui::printf(frame, control_refx - 75 + 20, controls_y + 55 + 43, 0.5,
                 0xFFFFFF, "%1.1f", data.pose.pose.position.x);

    // Create window at (40, 20) with size 250x80 (width x height) and title
    cvui::window(frame, control_refx, controls_y + 55, 70, 70, "Y");
    // Show how many times the button has been clicked inside the window.
    cvui::printf(frame, control_refx + 20, controls_y + 55 + 43, 0.5, 0xFFFFFF,
                 "%1.1f", data.pose.pose.position.y);

    // Create window at (40, 20) with size 250x80 (width x height) and title
    cvui::window(frame, control_refx + 75, controls_y + 55, 70, 70, "Z");
    // Show how many times the button has been clicked inside the window.
    cvui::printf(frame, control_refx + 75 + 20, controls_y + 55 + 43, 0.5,
                 0xFFFFFF, "%1.1f", data.pose.pose.position.z);
    //--------------------------------------------------------------------//

    cvui::window(frame, control_refx, controls_y + 135, 145, 70,
                 "Service: get distance");
    if (cvui::button(frame, control_refx - 75, controls_y + 135, but_size,
                     but_size, "Call")) {
      if (service_client.call(srv_req)) {
        // Print the response message and return true
        ROS_DEBUG("Response message: %s", srv_req.response.message.c_str());
        // set latest service call status
        last_service_call_msg = srv_req.response.message;

      } else {
        last_service_call_msg = "Service call failed.";
      }
    }

    if (not last_service_call_msg.empty()) {
      cvui::printf(frame, control_refx + 15, controls_y + 135 + 40, 1, 0xFFFFFF,
                   "%s", last_service_call_msg.c_str());
    }

    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }

    ros::spinOnce();
    rate.sleep();
  }
}
