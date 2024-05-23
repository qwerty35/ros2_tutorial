#ifndef ROS2_TUTORIAL_SIMULATOR_H
#define ROS2_TUTORIAL_SIMULATOR_H

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

class State {
public:
    double x = 0;
    double y = 0;
    double theta = 0;
};

class Input {
public:
    double v = 0;
    double w = 0;
};

class Simulator : public rclcpp::Node {
public:
    Simulator();

private:
    // ROS
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pose;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // Parameters
    double robot_scale = 0.5;
    double offset = 1.0;
    size_t number_of_robots = 5;
    double dt = 0.01;


    // States and inputs
    State state;
    Input cmd_vel;

    // Callback functions
    void timer_callback();

    void cmd_callback(const geometry_msgs::msg::Twist &msg);

    void update_state();

    void publish_marker_pose();

    void broadcast_tf();
};


#endif //ROS2_TUTORIAL_SIMULATOR_H
