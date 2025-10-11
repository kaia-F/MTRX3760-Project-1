#ifndef H_TES_FOLLOWER
#define H_TES_FOLLOWER

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <mutex> 

/**
 * @file turtlebot3_camera.hpp
 * @brief Camera_Node for Turtlebot3
 * 
 * This class encapsulates the subscribers, publishers, and helper functions
 * for Turtlebot to determine whether or not it has reached the goal state.
 */

class CameraNode : public rclcpp::Node
{
    public:
        CameraNode();

        
        bool get_latest_frames(cv::Mat &camera_frame, cv::Mat &mask_frame);

    private:

        // Callback function for image subscription
        void image_callback(const sensor_msgs::msg::Image::SharedPtr p_msg);

        // Checks to see if the goal state is reached and publishes if goal is reached
        void check_goal(const sensor_msgs::msg::Image::SharedPtr p_msg);


        // ROS publishers and subscribers
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_stop_pub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_sub;

        // Mutex and Mat objects for handling threading
        std::mutex image_mutex_;
        cv::Mat latest_camera_frame_;
        cv::Mat latest_mask_frame_;
};

#endif