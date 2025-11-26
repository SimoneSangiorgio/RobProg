#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// ===================================================================
// SENSORS
// ===================================================================
struct SensorConfig {
    std::string type;
    std::string frame_id;
    std::string topic;
    double relative_pose[3]; // x, y, theta
    int beams;
    double range_min, range_max;
    double angle_min, angle_max;
};

// ===================================================================
// ROBOT (da YAML)
// ===================================================================

struct RobotConfig {
    std::string name;
    std::string frame_id;
    double initial_pose[3]; // x, y, theta
    double max_linear_vel;
    double max_angular_vel;
    std::vector<SensorConfig> sensors;
    std::string odom_frame_id;
};


struct RobotState {
    RobotConfig config;
    double x, y, theta; // posizione attuale
    double current_linear_vel;
    double current_angular_vel;
    ros::Subscriber cmd_vel_sub;
    std::vector<ros::Publisher> sensor_pubs;
    ros::Publisher odom_pub;
};
