#ifndef H_POSE_TRAJECTORY_NODE
#define H_POSE_TRAJECTORY_NODE

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>

class PoseTrajectory : public rclcpp::Node
{
    public:
        PoseTrajectory();

    private:

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr p_msg);

        void track_position(geometry_msgs::msg::Point p_position);

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
        visualization_msgs::msg::Marker trajectory_marker;
};

#endif