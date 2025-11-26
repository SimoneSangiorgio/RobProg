#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>
#include "robot.h"

class Simulator {
public:
    Simulator(ros::NodeHandle& nh);
    void run();

private:
    void loadParameters();
    void loadMap();
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, int robot_index);
    void update(const ros::Duration& dt);
    void publishState(const ros::Time& now); 
    void simulateAndPublishLaser(RobotState& robot, const ros::Time& now);

    ros::NodeHandle nh_;
    std::vector<RobotState> robots_;
    nav_msgs::OccupancyGrid map_grid_;
    
    ros::Publisher map_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string map_image_path_;
    double map_resolution_;
    double map_origin_[3];
};
