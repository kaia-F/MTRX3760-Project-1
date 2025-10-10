#ifndef H_TES_FOLLOWER
#define H_TES_FOLLOWER

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <mutex> 

class CameraNode : public rclcpp::Node
{
    public:
        CameraNode();

        bool get_latest_frames(cv::Mat &camera_frame, cv::Mat &mask_frame);

    private:

        void image_callback(const sensor_msgs::msg::Image::SharedPtr p_msg);


        void check_goal(const sensor_msgs::msg::Image::SharedPtr p_msg);


        // ROS publishers and subscribers
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_stop_pub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_sub;
        //rclcpp::Publisher<std_msgs::msg::bool>::SharedPtr m_dash_pub;

        std::mutex image_mutex_;
        cv::Mat latest_camera_frame_;
        cv::Mat latest_mask_frame_;
};

#endif