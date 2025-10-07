// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

#include <memory>

using namespace std::chrono_literals;


double normalise_angle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}


// Constructor
Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
// Calls parent constructor Node with name "turtlebot3_drive_node".
// Registers this ROS 2 node.
  /************************************************************
  ** Initialise variables
  ************************************************************/
  // scan_data_ = LIDAR readings at three angles (front, left, right).
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;

  
  
	// robot_pose_ = robot’s yaw (heading).
  robot_pose_ = 0.0;
  // prev_robot_pose_ = yaw remembered before a turn (so robot knows when to stop turning).
  prev_robot_pose_ = 0.0;

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  // QoS (Quality of Service)
  // ROS 2 communication reliability setup: keeps last 10 messages in buffer.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  // Publishes velocity commands (linear.x, angular.z) on /cmd_vel.
cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", qos); 

  // Initialise subscribers
  /*
  Subscribes to /scan topic (LaserScan).
	Uses SensorDataQoS() (high priority for sensor streams).
	When data arrives, runs scan_callback().
	*/

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &Turtlebot3Drive::scan_callback, \
      this, \
      std::placeholders::_1));
  /*
  Subscribes to /odom (robot odometry).
	Calls odom_callback() when a new message arrives.
	*/
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  /*
  Every 10 ms, update_callback() runs.
  This is the “brain loop” (finite state machine).
	*/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&Turtlebot3Drive::update_callback, this));
	
	// Prints an info message once the node starts.
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialised");
}

// Destructor
// Runs when program exits. Logs termination message.
Turtlebot3Drive::~Turtlebot3Drive()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
// Odometry Callback
/*
Called when /odom publishes a new message.
Reads quaternion orientation from odometry.
*/
void Turtlebot3Drive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
}

void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  uint16_t scan_angle[3] = {0, 90, 270};

  for (int num = 0; num < 3; num++) {
    if (std::isinf(msg->ranges.at(scan_angle[num]))) {
      scan_data_[num] = msg->range_max;
    } else {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3Drive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = this->get_clock()->now();
  cmd_vel.twist.linear.x = linear;
  cmd_vel.twist.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void Turtlebot3Drive::update_callback()
{
    static uint8_t state = GET_TB3_DIRECTION;

    const double DEG90 = 90.0 * DEG2RAD;
    const double forward_clear = 1;    // front free distance
    const double side_clear = 0.4;       // right free distance

    double front = scan_data_[CENTER];
    double right = scan_data_[RIGHT];
    double left = scan_data_[LEFT];

    bool no_right_wall = (right > side_clear);
    bool no_front_wall = (front > forward_clear);
    
	switch (state)
    {
        case GET_TB3_DIRECTION:
            // Right-wall rule: try right first, then forward, else left
            if (no_right_wall)
            {
                RCLCPP_INFO(this->get_logger(), "[ACTION] Turning RIGHT - no wall detected on right");
                prev_robot_pose_ = robot_pose_;
                state = TB3_RIGHT_TURN;
            }
            else if (no_front_wall)
            {
                RCLCPP_INFO(this->get_logger(), "[ACTION] Moving FORWARD - path clear");
                state = TB3_DRIVE_FORWARD;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "[ACTION] Turning LEFT - wall ahead and on right");
                prev_robot_pose_ = robot_pose_;
                state = TB3_LEFT_TURN;
            }
            break;

        case TB3_DRIVE_FORWARD:
            RCLCPP_INFO(this->get_logger(), "[MOVING] Driving forward...");
            update_cmd_vel(LINEAR_VELOCITY, 0.0);
            state = GET_TB3_DIRECTION;
            break;

        case TB3_RIGHT_TURN:
		{
            // Turn 90° clockwise
            double angle_diff = normalise_angle(robot_pose_ - prev_robot_pose_);
            RCLCPP_INFO(this->get_logger(), "[TURNING] Right turn... angle_diff = %.2f rad", angle_diff);
            if (fabs(angle_diff) >= DEG90)
            {
                RCLCPP_INFO(this->get_logger(), "[TURN DONE] Right 90° turn complete");
                update_cmd_vel(0.0, 0.0);
                prev_robot_pose_ = robot_pose_;
                state = TB3_DRIVE_FORWARD;
            }
            else
            {
                update_cmd_vel(0.0, -ANGULAR_VELOCITY);
            }
            break;
		}

        case TB3_LEFT_TURN:
		{
            // Turn 90° counter-clockwise
            double angle_diff = normalise_angle(robot_pose_ - prev_robot_pose_);
            RCLCPP_INFO(this->get_logger(), "[TURNING] Left turn... angle_diff = %.2f rad", angle_diff);
			if (fabs(angle_diff) >= DEG90)
            {
				RCLCPP_INFO(this->get_logger(), "[TURN DONE] Left 90° turn complete");
                prev_robot_pose_ = robot_pose_;
                state = TB3_DRIVE_FORWARD;
            }
            else
            {
                update_cmd_vel(0.0, ANGULAR_VELOCITY);
            }
            break;
		}
        default:
			RCLCPP_WARN(this->get_logger(), "[WARN] Unknown state, resetting...");
            state = GET_TB3_DIRECTION;
            break;
    }
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();

  return 0;
}

