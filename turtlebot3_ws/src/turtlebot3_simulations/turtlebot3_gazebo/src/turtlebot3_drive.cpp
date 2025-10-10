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
  uint16_t scan_angle[3] = {0, 90, 270};

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
    static uint8_t state = TB3_DRIVE_FORWARD;
    static uint8_t prev_state = GET_TB3_DIRECTION;

    // LIDAR readings
    double front = sensor_data_.get_front();
    double left = sensor_data_.get_left();
    double right = sensor_data_.get_right();
    
    // Wall detection flags
    bool front_clear = (front > WallFollowerConfig::FRONT_OBSTACLE_DISTANCE);
    bool wall_on_right = (right < WallFollowerConfig::WALL_DETECTION_RANGE);
    bool wall_on_left = (left < WallFollowerConfig::WALL_DETECTION_RANGE);
    
    static double forward_start_x = 0.0;
    static double forward_start_y = 0.0;

    if (!sensor_data_.is_ready())    
    {
        // LIDAR not ready — just stop and wait
        update_cmd_vel(0.0, 0.0);
        return;
    }

    // Log only when state changes
    if (state != prev_state)
    {
        const char* state_names[] = {"DECISION", "FORWARD", "RIGHT_TURN", "LEFT_TURN", "COMMIT_FWD"};
        RCLCPP_INFO(this->get_logger(), "\n========================================");
        RCLCPP_INFO(this->get_logger(), "[STATE CHANGE] %s -> %s", 
            state_names[prev_state], state_names[state]);
        RCLCPP_INFO(this->get_logger(), "[SENSORS] Front:%.2f Left:%.2f Right:%.2f", front, left, right);
        prev_state = state;
    }
    
    switch (state)
    {
        case GET_TB3_DIRECTION:
        {
            // Right-wall following priority:
            // 1. If no wall on right -> turn right
            // 2. If front blocked -> turn left
            // 3. Otherwise -> follow wall forward
            
            if (wall_on_left && !wall_on_right)
            {
                // Special case: wall on left, no wall on right at startup
                RCLCPP_INFO(this->get_logger(), "Wall on left only — performing 180° right turn");
                robot_state_.save_yaw();
                state = TB3_RIGHT_TURN_180;
            }
            else if (!wall_on_right)
            {
                // Lost the wall, turn right to find it
                //RCLCPP_INFO(this->get_logger(), "[DECISION] No right wall detected - turning RIGHT");
                robot_state_.save_yaw();
                forward_start_x = robot_state_.get_x();
                forward_start_y = robot_state_.get_y();
                state = TB3_PRE_RIGHT_COMMIT;
            }
            else if (!front_clear)
            {
                // Obstacle ahead, turn left
                //RCLCPP_INFO(this->get_logger(), "[DECISION] Front blocked (%.2fm) - turning LEFT", front);
                robot_state_.save_yaw();
                state = TB3_LEFT_TURN;
            }
            else
            {
                // Follow the wall
                //RCLCPP_INFO(this->get_logger(), "[DECISION] Wall on right, front clear - moving FORWARD");
                state = TB3_DRIVE_FORWARD;
            }
            break;
        }

        case TB3_DRIVE_FORWARD:
        {
            // SMOOTH wall following with proportional control + WallFollowerConfig::CONTROL_DEADBAND
            double linear = WallFollowerConfig::LINEAR_VELOCITY;
            double angular = 0.0;
            
            if (wall_on_right)
            {
                // Calculate error: positive = too far, negative = too close
                double error = right - WallFollowerConfig::TARGET_WALL_DISTANCE;
                
                // WallFollowerConfig::CONTROL_DEADBAND: Ignore small errors to prevent oscillation
                if (fabs(error) > WallFollowerConfig::CONTROL_DEADBAND)
                {
                    // Proportional control: steer toward wall if too far, away if too close
                    angular = -WallFollowerConfig::PROPORTIONAL_GAIN * error;  // Negative because right wall
                    
                    // Limit angular velocity to prevent oscillation
                    if (angular > WallFollowerConfig::MAX_ANGULAR_VELOCITY) angular = WallFollowerConfig::MAX_ANGULAR_VELOCITY;
                    if (angular < -WallFollowerConfig::MAX_ANGULAR_VELOCITY) angular = -WallFollowerConfig::MAX_ANGULAR_VELOCITY;
                    
                    // Reduce linear speed during corrections
                    if (fabs(angular) > WallFollowerConfig::CORRECTION_THRESHOLD)
                    {
                        linear = WallFollowerConfig::LINEAR_VELOCITY * WallFollowerConfig::CORRECTION_SPEED_FACTOR;  
                    }
                }
                // else: within WallFollowerConfig::CONTROL_DEADBAND, go straight (angular stays 0.0)
            }
            
            update_cmd_vel(linear, angular);
            
            // Check if conditions changed
            if (!front_clear || !wall_on_right)
            {
                state = GET_TB3_DIRECTION;
            }
            break;
        }

        case TB3_RIGHT_TURN:
        {
            // Turn 90° clockwise (negative angular velocity)
            double angle_diff = robot_state_.get_angle_turned(normalise_angle);
            
            //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200, "[RIGHT TURN] Progress: %.1f°", fabs(angle_diff) * RAD2DEG);
            
            if (fabs(angle_diff) >= (WallFollowerConfig::TURN_ANGLE_90 - WallFollowerConfig::ANGLE_TOLERANCE))
            {
                //RCLCPP_INFO(this->get_logger(), "[TURN COMPLETE] Right turn finished - committing forward");
                update_cmd_vel(0.0, 0.0);

                // Record where we finished the turn
                forward_start_x = robot_state_.get_x();
                forward_start_y = robot_state_.get_y(); 

                // Enter commit forward state
                state = TB3_POST_RIGHT_COMMIT;

            }
            else
            {
                update_cmd_vel(0.0, -WallFollowerConfig::ANGULAR_VELOCITY);
            }
            break;
        }

        case TB3_LEFT_TURN:
        {
            // Turn 90° counter-clockwise (positive angular velocity)
            double angle_diff = robot_state_.get_angle_turned(normalise_angle);
            
            //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200, "[LEFT TURN] Progress: %.1f°", fabs(angle_diff) * RAD2DEG);
            
            if (fabs(angle_diff) >= (WallFollowerConfig::TURN_ANGLE_90 - WallFollowerConfig::ANGLE_TOLERANCE))
            {
                RCLCPP_INFO(this->get_logger(), "[TURN COMPLETE] Left turn finished - returning to decision");
                update_cmd_vel(0.0, 0.0);
                state = GET_TB3_DIRECTION;
            }
            else
            {
                update_cmd_vel(0.0, WallFollowerConfig::ANGULAR_VELOCITY);
            }
            break;
        }

        case TB3_RIGHT_TURN_180:
        {
            double angle_diff = robot_state_.get_angle_turned(normalise_angle);

            if (fabs(angle_diff) >= (WallFollowerConfig::TURN_ANGLE_180 - WallFollowerConfig::ANGLE_TOLERANCE))
            {
                update_cmd_vel(0.0, 0.0);
                state = TB3_DRIVE_FORWARD;
            }
            else
            {
                update_cmd_vel(0.0, WallFollowerConfig::ANGULAR_VELOCITY);
            }
            break;
        }

        case TB3_PRE_RIGHT_COMMIT:
        {
            const double distance = robot_state_.distance_to(forward_start_x, forward_start_y);

            if (distance >= WallFollowerConfig::PRE_TURN_COMMIT_DISTANCE)
            {
                update_cmd_vel(0.0, 0.0);
                robot_state_.save_yaw();
                state = TB3_RIGHT_TURN;
            }
            else
            {
                update_cmd_vel(WallFollowerConfig::LINEAR_VELOCITY, 0.0);
            }
            break;
        }
        case TB3_POST_RIGHT_COMMIT:
        {
            const double distance = robot_state_.distance_to(forward_start_x, forward_start_y);

            if (distance >= WallFollowerConfig::POST_TURN_COMMIT_DISTANCE)
            {
                update_cmd_vel(0.0, 0.0);
                state = GET_TB3_DIRECTION;
            }
            else
            {
                update_cmd_vel(WallFollowerConfig::LINEAR_VELOCITY, 0.0);
            }
            break;
        }
        default:
            RCLCPP_WARN(this->get_logger(), "[ERROR] Unknown state %d - resetting", state);
            update_cmd_vel(0.0, 0.0);
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

