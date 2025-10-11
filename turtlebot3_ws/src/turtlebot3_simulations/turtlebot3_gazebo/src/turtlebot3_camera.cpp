#include "turtlebot3_gazebo/turtlebot3_camera.hpp"
#include <thread>

CameraNode::CameraNode() : Node("turtlebot_camera_node")
{
    m_stop_pub = this->create_publisher<std_msgs::msg::Bool>("goal_reached_signal", 10);
    m_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 10,
        std::bind(&CameraNode::image_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Camera node initialized.");
}

// Public method for the main thread to get data
bool CameraNode::get_latest_frames(cv::Mat &camera_frame, cv::Mat &mask_frame)
{
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (latest_camera_frame_.empty() || latest_mask_frame_.empty())
    {
        return false;
    }
    camera_frame = latest_camera_frame_.clone();
    mask_frame = latest_mask_frame_.clone();
    return true;
}

// This callback simply directs the message to the processing function.
// It runs on the background ROS thread.
void CameraNode::image_callback(const sensor_msgs::msg::Image::SharedPtr p_msg)
{
    check_goal(p_msg);
}

// This function processes the image data.
// It runs on the background ROS thread. NO GUI CALLS ARE ALLOWED HERE.
void CameraNode::check_goal(const sensor_msgs::msg::Image::SharedPtr p_msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(p_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        // Set
        cv::Scalar lower_bound(0, 100, 0);
        cv::Scalar upper_bound(25, 255, 25);
        cv::Mat green_mask;
        cv::inRange(hsv_image, lower_bound, upper_bound, green_mask);

        const double green_area = cv::countNonZero(green_mask);
        const double total_area = green_mask.rows * green_mask.cols;
        const double percent_green = green_area / total_area;
        bool goal_reached = (percent_green > 0.8);

        std_msgs::msg::Bool stop_signal;
        stop_signal.data = goal_reached;
        m_stop_pub->publish(stop_signal);

        // Safely store the processed images for the main GUI thread
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_camera_frame_ = cv_ptr->image;
            latest_mask_frame_ = green_mask;
        }
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

// The main function: Manages threads and the GUI
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto camera_node = std::make_shared<CameraNode>();

    // Create a separate thread to run all ROS operations
    std::thread ros_thread([&]() {
        rclcpp::spin(camera_node);
    });

    cv::namedWindow("Camera Feed");
    cv::namedWindow("Green Detection");
    cv::Mat camera_feed;
    cv::Mat green_detection_feed;

    while (rclcpp::ok())
    {
        // Safely get the latest processed frames
        if (camera_node->get_latest_frames(camera_feed, green_detection_feed))
        {
            cv::imshow("Camera Feed", camera_feed);
            cv::imshow("Green Detection", green_detection_feed);
        }

        // Handle GUI events and allow windows to update
        if (cv::waitKey(10) == 27) // Exit if 'ESC' is pressed
        {
            break;
        }
    }

    // Ensure a clean shutdown
    rclcpp::shutdown();
    if (ros_thread.joinable())
    {
        ros_thread.join();
    }
    cv::destroyAllWindows();
    return 0;
}