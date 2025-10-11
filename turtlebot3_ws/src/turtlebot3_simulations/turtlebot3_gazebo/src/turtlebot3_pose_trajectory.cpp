#include <turtlebot3_gazebo/turtlebot3_pose_trajectory.hpp>

PoseTrajectory::PoseTrajectory() : Node("pose_trajectory_node")
{
    // Initialize subscriber to robot odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&PoseTrajectory::odom_callback, this, std::placeholders::_1));
    
    trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "trajectory_marker", 10);

    // Set marker properties
    trajectory_marker.header.frame_id = "map";
    trajectory_marker.ns = "trajectory";
    trajectory_marker.id = 0;
    trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker.scale.x = 0.1;

    // Set marker color
    trajectory_marker.color.r = 1.0f;
    trajectory_marker.color.g = 0.0f;
    trajectory_marker.color.b = 0.0f;
    trajectory_marker.color.a = 1.0f;

    trajectory_marker.lifetime = rclcpp::Duration(0, 0);
}

rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
visualization_msgs::msg::Marker trajectory_marker;

// Odometry Callback
/*
Called when /odom publishes a new message.
Reads quaternion orientation from odometry.
*/
void PoseTrajectory::odom_callback(const nav_msgs::msg::Odometry::SharedPtr p_msg)
{
    // Extract position from Odometry message and track
    track_position(p_msg->pose.pose.position);
}

// Track Position
/*
Called by odom_callback to update the trajectory marker
*/
void PoseTrajectory::track_position(geometry_msgs::msg::Point p_position)
{
    geometry_msgs::msg::Point current_position;
    current_position.x = p_position.x;
    current_position.y = p_position.y;
    current_position.z = p_position.z;

    trajectory_marker.points.push_back(current_position);

    trajectory_marker.header.stamp = this->now();

    trajectory_pub_->publish(trajectory_marker);
}

int main(int argc, char **argv)
{
    // Create ROS node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseTrajectory>();

    // Spin ROS node
    rclcpp::spin(node);

    // Handle ROS node shutdown
    rclcpp::shutdown();
    return 0;
}