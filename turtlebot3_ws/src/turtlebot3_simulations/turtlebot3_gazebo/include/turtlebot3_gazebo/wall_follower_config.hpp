#ifndef TURTLEBOT3_GAZEBO__WALL_FOLLOWER_CONFIG_HPP_
#define TURTLEBOT3_GAZEBO__WALL_FOLLOWER_CONFIG_HPP_

#include <cmath>

/**
 * @brief Configuration parameters for wall-following behavior
 * 
 * Centralizes all tuning parameters to eliminate magic numbers
 * and make parameter adjustment easier.
 */
class WallFollowerConfig
{
public:
  // Velocity parameters
  static constexpr double LINEAR_VELOCITY = 0.3; 
  static constexpr double ANGULAR_VELOCITY = 0.9; //0.9 
  static constexpr double MAX_ANGULAR_VELOCITY = 0.35; 
  
  // Distance thresholds
  static constexpr double TARGET_WALL_DISTANCE = 0.36; //0.36
  static constexpr double WALL_DETECTION_RANGE = 0.7; //0.7
  static constexpr double FRONT_OBSTACLE_DISTANCE = 0.5; //0.5
  static constexpr double PRE_TURN_COMMIT_DISTANCE = 0.36; //0.36
  static constexpr double POST_TURN_COMMIT_DISTANCE = 0.45; //0.45
  
  // Control parameters
  static constexpr double PROPORTIONAL_GAIN = 0.5; 
  static constexpr double CONTROL_DEADBAND = 0.05; 
  static constexpr double CORRECTION_SPEED_FACTOR = 0.75; // Slow down 25% during corrections (0.75)
  static constexpr double CORRECTION_THRESHOLD = 0.2; 
  
  // Angle parameters
  static constexpr double TURN_ANGLE_90 = M_PI / 2.0; 
  static constexpr double TURN_ANGLE_180 = M_PI; 
  static constexpr double ANGLE_TOLERANCE = 0.03; 
  
  // Conversion constants
  static constexpr double DEG2RAD = M_PI / 180.0;
  static constexpr double RAD2DEG = 180.0 / M_PI;
};

#endif  // TURTLEBOT3_GAZEBO__WALL_FOLLOWER_CONFIG_HPP_