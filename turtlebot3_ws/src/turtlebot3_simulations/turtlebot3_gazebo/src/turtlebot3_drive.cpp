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
#include "turtlebot3_gazebo/wall_follower_config.hpp"
#include "turtlebot3_gazebo/wall_follower_state_machine.hpp"
#include "turtlebot3_gazebo/utilities.hpp"

#include <memory>

using namespace std::chrono_literals;


// Constructor
Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node"), goal_reached_(false)
{
// Calls parent constructor Node with name "turtlebot3_drive_node".
// Registers this ROS 2 node.
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
  
  /*
  Subscribes to /stop_signal topic
  Calls stop_callback() when a new message arrives.
  */
  stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "stop_signal", 10, std::bind(&Turtlebot3Drive::stop_callback, this, std::placeholders::_1));

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
  // Update position
  robot_state_.set_position(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y);

  // Convert quaternion to yaw
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_state_.set_yaw(yaw);
}


void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  uint16_t scan_angle[3] = {0, 80, 280};

  // Update front (CENTER = 0)
  if (std::isinf(msg->ranges.at(scan_angle[0]))) {
    sensor_data_.set_front(msg->range_max); 
  } else {
    sensor_data_.set_front(msg->ranges.at(scan_angle[0]));  
  }
  
  // Update left (LEFT = 1)
  if (std::isinf(msg->ranges.at(scan_angle[1]))) {
    sensor_data_.set_left(msg->range_max);  
  } else {
    sensor_data_.set_left(msg->ranges.at(scan_angle[1]));  
  }
  
  // Update right (RIGHT = 2)
  if (std::isinf(msg->ranges.at(scan_angle[2]))) {
    sensor_data_.set_right(msg->range_max);  
  } else {
    sensor_data_.set_right(msg->ranges.at(scan_angle[2]));  
  }
}

void Turtlebot3Drive::stop_callback(const std_msgs::msg::Bool::SharedPtr p_msg)
{
  if (p_msg->data)
  {
    goal_reached_ = true;
    RCLCPP_INFO(this->get_logger(), "Stop signal received. Halting robot motion");
    update_cmd_vel(0.0, 0.0);
    return;
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
  // Check if goal position has been reached 
  if (goal_reached_)
  {
    update_cmd_vel(0.0, 0.0);
    return;
  }
  
  // Check if sensors are ready
  if (!sensor_data_.is_ready())
  {
    update_cmd_vel(0.0, 0.0);
    return;
  }

  // Save previous state for logging
  static WallFollowerStateMachine::State prev_logged_state = WallFollowerStateMachine::GET_TB3_DIRECTION;
  WallFollowerStateMachine::State current_state_before = state_machine_.get_current_state();
  
  // Save yaw before state machine runs (needed for turn tracking)
  // Check which states need yaw saved
  if (current_state_before == WallFollowerStateMachine::GET_TB3_DIRECTION)
  {
    robot_state_.save_yaw();
  }
  else if (current_state_before == WallFollowerStateMachine::TB3_PRE_RIGHT_COMMIT)
  {
    double dist = robot_state_.distance_to(robot_state_.get_x(), robot_state_.get_y());
    if (dist >= WallFollowerConfig::PRE_TURN_COMMIT_DISTANCE - 0.01)
    {
      robot_state_.save_yaw();
    }
  }
  
  // Run the state machine
  VelocityCommand cmd = state_machine_.update(sensor_data_, robot_state_, normalise_angle);
  
  // Log state changes
  WallFollowerStateMachine::State current_state_after = state_machine_.get_current_state();
  if (current_state_after != prev_logged_state)
  {
    const char* state_names[] = {"DECISION", "FORWARD", "RIGHT_TURN", "LEFT_TURN", 
                                  "RIGHT_TURN_180", "PRE_RIGHT_COMMIT", "POST_RIGHT_COMMIT"};
    RCLCPP_INFO(this->get_logger(), "\n========================================");
    RCLCPP_INFO(this->get_logger(), "[STATE CHANGE] %s -> %s", 
      state_names[prev_logged_state], state_names[current_state_after]);
    RCLCPP_INFO(this->get_logger(), "[SENSORS] Front:%.2f Left:%.2f Right:%.2f", 
      sensor_data_.get_front(), sensor_data_.get_left(), sensor_data_.get_right());
    prev_logged_state = current_state_after;
  }
  
  // Send command to robot
  update_cmd_vel(cmd.linear, cmd.angular);
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

