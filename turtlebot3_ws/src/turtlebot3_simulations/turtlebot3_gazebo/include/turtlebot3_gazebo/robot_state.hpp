#ifndef TURTLEBOT3_GAZEBO__ROBOT_STATE_HPP_
#define TURTLEBOT3_GAZEBO__ROBOT_STATE_HPP_

#include <cmath>

/**
 * @file robot_state.hpp
 * @brief Encapsulates robot position and orientation
 * 
 * This class manages the robot's current pose (x, y, yaw) and provides
 * utility methods for distance calculations and angle operations.
 */

class RobotState
{
public:
  //Default constructor - initializes all values to zero
  RobotState()
  {
    robot_pose_ = 0.0;
    prev_robot_pose_ = 0.0;
    current_x_ = 0.0;
    current_y_ = 0.0;
  }
  
  //Get current yaw angle (heading)
  double get_yaw() const
  {
    return robot_pose_;
  }
  
  //Get previously saved yaw angle
  double get_prev_yaw() const
  {
    return prev_robot_pose_;
  }
  
  //Get current x position
  double get_x() const
  {
    return current_x_;
  }
  
  //Get current y position
  double get_y() const
  {
    return current_y_;
  }
  
  //Set current yaw angle
  void set_yaw(double yaw)
  {
    robot_pose_ = yaw;
  }
  
  //Save current yaw as previous yaw (for turn tracking)
  void save_yaw()
  {
    prev_robot_pose_ = robot_pose_;
  }
  
  //Set current position
  void set_position(double x, double y)
  {
    current_x_ = x;
    current_y_ = y;
  }
  
  //Calculate distance from current position to a target point
  double distance_to(double target_x, double target_y) const
  {
    double dx = current_x_ - target_x;
    double dy = current_y_ - target_y;
    return std::sqrt(dx * dx + dy * dy);
  }
  
  //Calculate angular difference from saved previous yaw
  double get_angle_turned(double (*normalise_angle)(double)) const
  {
    return normalise_angle(robot_pose_ - prev_robot_pose_);
  }

private:
  // Robot's current yaw (heading angle)
  double robot_pose_;
  
  // Previously saved yaw (used to track turn progress)
  double prev_robot_pose_;
  
  // Current x position
  double current_x_;
  
  // Current y position
  double current_y_;
};

#endif  // TURTLEBOT3_GAZEBO__ROBOT_STATE_HPP_