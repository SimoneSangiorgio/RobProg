#include <ros/ros.h>
#include <ros/package.h>
#include <XmlRpcValue.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h> 
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h> 
#include <opencv2/opencv.hpp>

#include "multi_robot_simulator/robot.h"
#include "multi_robot_simulator/simulator.h"

// ===================================================================

Simulator::Simulator(ros::NodeHandle& nh) : nh_(nh) {
    ROS_INFO("Inizializzazione del simulatore...");
    loadParameters();
    loadMap();

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

    for (int i = 0; i < robots_.size(); ++i) {
        std::string cmd_vel_topic = "/" + robots_[i].config.name + "/cmd_vel";
        robots_[i].cmd_vel_sub = nh_.subscribe<geometry_msgs::Twist>(
            cmd_vel_topic, 10, boost::bind(&Simulator::cmdVelCallback, this, _1, i));
        ROS_INFO("Sottoscritto al topic: %s", cmd_vel_topic.c_str());

        std::string odom_topic = "/" + robots_[i].config.name + "/odom";
        robots_[i].odom_pub = nh_.advertise<nav_msgs::Odometry>(odom_topic, 50);
        ROS_INFO("Pubblicherà dati di odometria su: %s", odom_topic.c_str());

        for (const auto& sensor_config : robots_[i].config.sensors) {
            ros::Publisher pub = nh_.advertise<sensor_msgs::LaserScan>(sensor_config.topic, 50);
            robots_[i].sensor_pubs.push_back(pub);
            ROS_INFO("Pubblicherà dati laser su: %s", sensor_config.topic.c_str());
        }
    }
}

void Simulator::loadParameters() {
    nh_.getParam("/map/image_path", map_image_path_);
    nh_.getParam("/map/resolution", map_resolution_);
    XmlRpc::XmlRpcValue origin_list;
    nh_.getParam("/map/origin", origin_list);
    for (int i=0; i<3; ++i) map_origin_[i] = origin_list[i];

    XmlRpc::XmlRpcValue robots_list;
    nh_.getParam("/robots", robots_list);

    for (int i=0; i<robots_list.size(); ++i) {
        RobotState robot_state;
        robot_state.config.name = (std::string)robots_list[i]["name"];
        robot_state.config.frame_id = (std::string)robots_list[i]["frame_id"];
        robot_state.config.max_linear_vel = robots_list[i]["max_velocities"]["linear_x"];
        robot_state.config.max_angular_vel = robots_list[i]["max_velocities"]["angular_z"];

        for (int j=0; j<3; ++j) robot_state.config.initial_pose[j] = robots_list[i]["initial_pose"][j];
        robot_state.x = robot_state.config.initial_pose[0];
        robot_state.y = robot_state.config.initial_pose[1];
        robot_state.theta = robot_state.config.initial_pose[2];
        robot_state.current_linear_vel = 0.0;
        robot_state.current_angular_vel = 0.0;
        
        robot_state.config.odom_frame_id = robot_state.config.name + "/odom";

        XmlRpc::XmlRpcValue sensors_list = robots_list[i]["sensors"];
        for (int j=0; j<sensors_list.size(); ++j) {
            SensorConfig sensor_cfg;
            sensor_cfg.type = (std::string)sensors_list[j]["type"];
            sensor_cfg.frame_id = (std::string)sensors_list[j]["frame_id"];
            sensor_cfg.topic = (std::string)sensors_list[j]["topic"];
            for (int k=0; k<3; ++k) sensor_cfg.relative_pose[k] = sensors_list[j]["relative_pose"][k];
            sensor_cfg.beams = (int)sensors_list[j]["beams"];
            sensor_cfg.range_min = sensors_list[j]["range_min"];
            sensor_cfg.range_max = sensors_list[j]["range_max"];
            sensor_cfg.angle_min = sensors_list[j]["angle_min"];
            sensor_cfg.angle_max = sensors_list[j]["angle_max"];
            robot_state.config.sensors.push_back(sensor_cfg);
        }
        robots_.push_back(robot_state);
        ROS_INFO("Caricato robot: %s", robot_state.config.name.c_str());
    }
}

void Simulator::loadMap() {
    std::string full_path;
    if (map_image_path_.find("$(find") != std::string::npos) {
        std::string pkg = "multi_robot_simulator";
        std::string package_path = ros::package::getPath(pkg);
        std::string file_rel = map_image_path_.substr(map_image_path_.find(")/") + 2);
        full_path = package_path + "/" + file_rel;
    } else {
        std::string package_path = ros::package::getPath("multi_robot_simulator");
        full_path = package_path + "/" + map_image_path_;
    }

    cv::Mat map_image = cv::imread(full_path, cv::IMREAD_GRAYSCALE);
    if (map_image.empty()) {
        ROS_ERROR("Impossibile caricare la mappa da: %s", full_path.c_str());
        ros::shutdown();
        return;
    }

    map_grid_.header.frame_id = "map";
    map_grid_.info.resolution = map_resolution_;
    map_grid_.info.width = map_image.cols;
    map_grid_.info.height = map_image.rows;
    map_grid_.info.origin.position.x = map_origin_[0];
    map_grid_.info.origin.position.y = map_origin_[1];
    tf2::Quaternion q;
    q.setRPY(0, 0, map_origin_[2]);
    map_grid_.info.origin.orientation.x = q.x();
    map_grid_.info.origin.orientation.y = q.y();
    map_grid_.info.origin.orientation.z = q.z();
    map_grid_.info.origin.orientation.w = q.w();

    map_grid_.data.resize(map_image.cols * map_image.rows);
    for (int y=0; y<map_image.rows; ++y) {
        for (int x=0; x<map_image.cols; ++x) {
            int inverted_y = map_image.rows - 1 - y;
            int pixel_val = map_image.at<uchar>(y, x);
            int map_index = inverted_y * map_image.cols + x;

            if (pixel_val < 127) map_grid_.data[map_index] = 100;
            else if (pixel_val > 127) map_grid_.data[map_index] = 0;
            else map_grid_.data[map_index] = -1;
        }
    }
    ROS_INFO("Mappa caricata con successo (%d x %d).", map_grid_.info.width, map_grid_.info.height);
}

void Simulator::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, int robot_index) {
    robots_[robot_index].current_linear_vel = std::max(-robots_[robot_index].config.max_linear_vel,
        std::min(robots_[robot_index].config.max_linear_vel, msg->linear.x));
    robots_[robot_index].current_angular_vel = std::max(-robots_[robot_index].config.max_angular_vel,
        std::min(robots_[robot_index].config.max_angular_vel, msg->angular.z));
}

void Simulator::update(const ros::Duration& dt) {
    double delta_time = dt.toSec();
    for (auto& robot : robots_) {
        double delta_x = robot.current_linear_vel * cos(robot.theta) * delta_time;
        double delta_y = robot.current_linear_vel * sin(robot.theta) * delta_time;
        double delta_theta = robot.current_angular_vel * delta_time;
        robot.x += delta_x;
        robot.y += delta_y;
        robot.theta += delta_theta;
    }
}

void Simulator::publishState(const ros::Time& now) {
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (auto& robot : robots_) {
        tf2::Quaternion q;
        q.setRPY(0, 0, robot.theta);

        geometry_msgs::TransformStamped odom_transform;
        odom_transform.header.stamp = now;

        odom_transform.header.frame_id = robot.config.odom_frame_id; 
        odom_transform.child_frame_id = robot.config.frame_id;
        odom_transform.transform.translation.x = robot.x;
        odom_transform.transform.translation.y = robot.y;
        odom_transform.transform.translation.z = 0.0;
        odom_transform.transform.rotation.x = q.x();
        odom_transform.transform.rotation.y = q.y();
        odom_transform.transform.rotation.z = q.z();
        odom_transform.transform.rotation.w = q.w();
        transforms.push_back(odom_transform);

        for (const auto& sensor : robot.config.sensors) {
            geometry_msgs::TransformStamped sensor_tf;
            sensor_tf.header.stamp = now;
            sensor_tf.header.frame_id = robot.config.frame_id;
            sensor_tf.child_frame_id = sensor.frame_id;
            sensor_tf.transform.translation.x = sensor.relative_pose[0];
            sensor_tf.transform.translation.y = sensor.relative_pose[1];
            sensor_tf.transform.translation.z = 0.0;
            tf2::Quaternion sq;
            sq.setRPY(0, 0, sensor.relative_pose[2]);
            sensor_tf.transform.rotation.x = sq.x();
            sensor_tf.transform.rotation.y = sq.y();
            sensor_tf.transform.rotation.z = sq.z();
            sensor_tf.transform.rotation.w = sq.w();
            transforms.push_back(sensor_tf);
        }
        
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = robot.config.odom_frame_id;
        odom_msg.child_frame_id = robot.config.frame_id;

        odom_msg.pose.pose.position.x = robot.x;
        odom_msg.pose.pose.position.y = robot.y;
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = robot.current_linear_vel;
        odom_msg.twist.twist.angular.z = robot.current_angular_vel;

        robot.odom_pub.publish(odom_msg);
    }

    tf_broadcaster_.sendTransform(transforms);
}


void Simulator::simulateAndPublishLaser(RobotState& robot, const ros::Time& now) {

    for (int i=0; i<robot.config.sensors.size(); ++i) {
        const auto& sensor_cfg = robot.config.sensors[i];

        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = now;
        scan_msg.header.frame_id = sensor_cfg.frame_id;
        scan_msg.angle_min = sensor_cfg.angle_min;
        scan_msg.angle_max = sensor_cfg.angle_max;
        scan_msg.angle_increment = (sensor_cfg.angle_max - sensor_cfg.angle_min) / (sensor_cfg.beams - 1);
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = 0.1;
        scan_msg.range_min = sensor_cfg.range_min;
        scan_msg.range_max = sensor_cfg.range_max;
        scan_msg.ranges.resize(sensor_cfg.beams);

        double sensor_global_x = robot.x + sensor_cfg.relative_pose[0] * cos(robot.theta) - sensor_cfg.relative_pose[1] * sin(robot.theta);
        double sensor_global_y = robot.y + sensor_cfg.relative_pose[0] * sin(robot.theta) + sensor_cfg.relative_pose[1] * cos(robot.theta);
        double sensor_global_theta = robot.theta + sensor_cfg.relative_pose[2];

        for (int j=0; j<sensor_cfg.beams; ++j) {
            double current_angle = sensor_global_theta + sensor_cfg.angle_min + j * scan_msg.angle_increment;
            double ray_dist = 0;
            bool hit = false;
            for (double dist=0; dist<sensor_cfg.range_max; dist += map_resolution_ / 2.0) {
                double check_x = sensor_global_x + dist * cos(current_angle);
                double check_y = sensor_global_y + dist * sin(current_angle);

                int map_x = (check_x - map_origin_[0]) / map_resolution_;
                int map_y = (check_y - map_origin_[1]) / map_resolution_;

                if (map_x >=0 && map_x < map_grid_.info.width && map_y >=0 && map_y < map_grid_.info.height) {
                    if (map_grid_.data[map_y * map_grid_.info.width + map_x] == 100) {
                        ray_dist = dist;
                        hit = true;
                        break;
                    }
                }
            }
            if (hit && ray_dist >= sensor_cfg.range_min) scan_msg.ranges[j] = ray_dist;
            else scan_msg.ranges[j] = std::numeric_limits<float>::infinity();
        }
        robot.sensor_pubs[i].publish(scan_msg);
    }
}

void Simulator::run() {
    ros::Rate loop_rate(50);
    ros::Time last_time = ros::Time::now();

    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        ros::Duration dt = now - last_time;
        last_time = now;

        update(dt);

        publishState(now); 
        for (auto& robot : robots_) {
            simulateAndPublishLaser(robot, now);
        }
        map_pub_.publish(map_grid_);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// ===================================================================
// MAIN
// ===================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_robot_simulator_node");
    ros::NodeHandle nh;
    Simulator sim(nh);
    sim.run();
    return 0;
}
