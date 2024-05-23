#ifndef ROS2_TUTORIAL_CMD_PUBLISHER_H
#define ROS2_TUTORIAL_CMD_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class CmdPublisher : public rclcpp::Node {
public:
  CmdPublisher();

private:
  void timer_tf_callback();

  void timer_cmd_callback();

  // Parameters
  double dt = 0.01;
  double dist_threshold = 0.05;
  double theta_threshold = 0.01;
  const double pi = 3.141592;

  // Robot state
//  double ori = 5.544444561004639;
  double ori = 0;
  double real_x = 0;
  double real_y = 0;
  double real_theta = 0;

  // Goal state
  double goal_x = 2;
  double goal_y = 0;
  double goal_theta = 0;

  // Mission state and parameter
  int count = 0;

  // PID error
  double prev_err_dist = 0;
  double prev_err_theta = 0;
  double sum_err_dist = 0;
  double sum_err_theta = 0;

  // PID gain
  double kP_dist = 1;
  double kI_dist = 0.05;
  double kD_dist = 0.01;
  double kP_theta = 1;
  double kI_theta = 0.05;
  double kD_theta = 0.01;

  rclcpp::TimerBase::SharedPtr timer_cmd, timer_tf;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_pose;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  // Init variables
  geometry_msgs::msg::Twist cmd_vel;
};

#endif //ROS2_TUTORIAL_CMD_PUBLISHER_H