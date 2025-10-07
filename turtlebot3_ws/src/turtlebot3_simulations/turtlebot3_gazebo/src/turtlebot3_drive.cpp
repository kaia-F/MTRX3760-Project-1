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
  
  current_x_ = 0.0;
  current_y_ = 0.0;

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
  // Get position
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

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
    static uint8_t prev_state = GET_TB3_DIRECTION;
    static double forward_start_x = 0.0;
    static double forward_start_y = 0.0;

    
    // Tuning parameters
    const double WALL_FOLLOW_DISTANCE = 0.3;  // Ideal distance from right wall
    const double WALL_DETECT_RANGE = 0.6;     // Max range to detect wall
    const double FRONT_OBSTACLE_DIST = 0.4;   // Min clear distance ahead
    const double COMMIT_FORWARD_DIST = 0.5;   // Distance to travel after right turn
    const double DEG90 = M_PI / 2.0;
    const double ANGLE_TOLERANCE = 0.1;       // Tolerance for turn completion (radians)
    
    // LIDAR readings
    double front = scan_data_[CENTER];
    double left = scan_data_[LEFT];
    double right = scan_data_[RIGHT];
    
    // Wall detection flags
    bool front_clear = (front > FRONT_OBSTACLE_DIST);
    bool wall_on_right = (right < WALL_DETECT_RANGE);
    
    // Log only when state changes
    if (state != prev_state)
    {
        const char* state_names[] = {"DECISION", "FORWARD", "RIGHT_TURN", "LEFT_TURN", "COMMIT_FWD"};
        RCLCPP_INFO(this->get_logger(), "\n========================================");
        RCLCPP_INFO(this->get_logger(), "[STATE CHANGE] %s -> %s", 
            state_names[prev_state], state_names[state]);
        RCLCPP_INFO(this->get_logger(), "[SENSORS] Front:%.2f Left:%.2f Right:%.2f", front, left, right);
        //RCLCPP_INFO(this->get_logger(), "========================================");
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
            
            if (!wall_on_right)
            {
                // Lost the wall, turn right to find it
                //RCLCPP_INFO(this->get_logger(), "[DECISION] No right wall detected - turning RIGHT");
                prev_robot_pose_ = robot_pose_;
                state = TB3_RIGHT_TURN;
            }
            else if (!front_clear)
            {
                // Obstacle ahead, turn left
                //RCLCPP_INFO(this->get_logger(), "[DECISION] Front blocked (%.2fm) - turning LEFT", front);
                prev_robot_pose_ = robot_pose_;
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
            double linear_vel = LINEAR_VELOCITY;
            double angular_vel = 0.0;
            
            //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "[MOVING] Forward | Front: %.2fm, Right: %.2fm", front, right);
            
            update_cmd_vel(linear_vel, angular_vel);
            
            // Check if we need to make a decision
            if (!front_clear || !wall_on_right)
            {
                RCLCPP_INFO(this->get_logger(), "[CHECK] Conditions changed - returning to decision state");
                state = GET_TB3_DIRECTION;
            }
            break;
        }

        case TB3_RIGHT_TURN:
        {
            // Turn 90° clockwise (negative angular velocity)
            double angle_diff = normalise_angle(robot_pose_ - prev_robot_pose_);
            
            //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200, "[RIGHT TURN] Progress: %.1f°", fabs(angle_diff) * RAD2DEG);
            
            if (fabs(angle_diff) >= (DEG90 - ANGLE_TOLERANCE))
            {
                //RCLCPP_INFO(this->get_logger(), "[TURN COMPLETE] Right turn finished - committing forward");
                update_cmd_vel(0.0, 0.0);
                
                // Record starting position for forward commitment
                forward_start_x = current_x_;
                forward_start_y = current_y_;
                
                state = TB3_DRIVE_FORWARD_COMMIT;
            }
            else
            {
                update_cmd_vel(0.0, -ANGULAR_VELOCITY);
            }
            break;
        }

        case TB3_LEFT_TURN:
        {
            // Turn 90° counter-clockwise (positive angular velocity)
            double angle_diff = normalise_angle(robot_pose_ - prev_robot_pose_);
            
            //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200, "[LEFT TURN] Progress: %.1f°", fabs(angle_diff) * RAD2DEG);
            
            if (fabs(angle_diff) >= (DEG90 - ANGLE_TOLERANCE))
            {
                RCLCPP_INFO(this->get_logger(), "[TURN COMPLETE] Left turn finished - returning to decision");
                update_cmd_vel(0.0, 0.0);
                state = GET_TB3_DIRECTION;
            }
            else
            {
                update_cmd_vel(0.0, ANGULAR_VELOCITY);
            }
            break;
        }

        case TB3_DRIVE_FORWARD_COMMIT:
        {
            // After right turn, commit to moving forward a set distance
            double distance_traveled = sqrt(
                pow(current_x_ - forward_start_x, 2) + 
                pow(current_y_ - forward_start_y, 2)
            );
            
            //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200, "[COMMIT FORWARD] Distance: %.2fm / %.2fm | Front: %.2fm", distance_traveled, COMMIT_FORWARD_DIST, front);
            
            // Emergency stop if wall detected ahead
            if (!front_clear)
            {
                //RCLCPP_WARN(this->get_logger(), "[COMMIT ABORT] Wall detected ahead - stopping early");
                update_cmd_vel(0.0, 0.0);
                state = GET_TB3_DIRECTION;
            }
            else if (distance_traveled >= COMMIT_FORWARD_DIST)
            {
                //RCLCPP_INFO(this->get_logger(), "[COMMIT DONE] Traveled %.2fm - returning to decision", distance_traveled);
                update_cmd_vel(0.0, 0.0);
                state = GET_TB3_DIRECTION;
            }
            else
            {
                // Keep moving forward
                update_cmd_vel(LINEAR_VELOCITY, 0.0);
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

