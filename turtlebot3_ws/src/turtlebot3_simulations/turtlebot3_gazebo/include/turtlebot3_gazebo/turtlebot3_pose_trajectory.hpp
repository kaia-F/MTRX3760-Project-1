#ifndef H_POSE_TRAJECTORY_NODE
#define H_POSE_TRAJECTORY_NODE

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>

/**
 * @file turtlebot3_pose_trajectory.hpp
 * @brief Trajectory recoding for the Turtlebot
 * 
 * This class encapsulates the publishing and subscribing
 * for the pose trajectory tracking. 
 */


class PoseTrajectory : public rclcpp::Node
{
    public:
        PoseTrajectory();

    private:
        
        // Callback function for Odometry
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr p_msg);

        // Takes the current point from Odometry and publishes and tracks it
        void track_position(geometry_msgs::msg::Point p_position);

        // ROS publishers and subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
        visualization_msgs::msg::Marker trajectory_marker;
};

#endif