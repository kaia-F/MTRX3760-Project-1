#ifndef TURTLEBOT3_GAZEBO__WALL_FOLLOWER_STATE_MACHINE_HPP_
#define TURTLEBOT3_GAZEBO__WALL_FOLLOWER_STATE_MACHINE_HPP_

#include "sensor_data.hpp"
#include "robot_state.hpp"
#include "wall_follower_config.hpp"
#include <cmath>

/**
 * @file wall_follower_state_machine.hpp
 * @brief State machine for wall-following navigation
 * 
 * This class encapsulates the decision-making logic for right-wall following.
 * It takes sensor readings and robot state as input, and returns velocity commands.
 */

//Simple structure to hold velocity commands
struct VelocityCommand
{
  double linear;   // Forward/backward velocity (m/s)
  double angular;  // Rotational velocity (rad/s)
  
  VelocityCommand() : linear(0.0), angular(0.0) {}
  VelocityCommand(double lin, double ang) : linear(lin), angular(ang) {}
};

class WallFollowerStateMachine
{
public:
  // State definitions
  enum State
  {
    GET_TB3_DIRECTION = 0,
    TB3_DRIVE_FORWARD = 1,
    TB3_RIGHT_TURN = 2,
    TB3_LEFT_TURN = 3,
    TB3_RIGHT_TURN_180 = 4,
    TB3_PRE_RIGHT_COMMIT = 5,
    TB3_POST_RIGHT_COMMIT = 6
  };
  
  // Constructor - initializes state machine
  WallFollowerStateMachine();
  
  /**
   * @brief Execute one cycle of the state machine
   * @param sensors Current sensor readings (front, left, right distances)
   * @param robot Current robot state (position and orientation)
   * @param normalise_angle Function pointer to normalize angles
   * @return Velocity command (linear and angular velocities)
   */
  VelocityCommand update(const SensorData& sensors, 
                         const RobotState& robot,
                         double (*normalise_angle)(double));
  
  //Get current state (for debugging/logging)
  State get_current_state() const { return current_state_; }
  
  //Get previous state (for detecting state changes)
  State get_previous_state() const { return previous_state_; }

private:
  // Current state of the finite state machine
  State current_state_;
  
  // Previous state (used to detect state changes for logging)
  State previous_state_;
  
  // Saved positions for distance tracking during commit phases
  double forward_start_x_;
  double forward_start_y_;
};

#endif  // TURTLEBOT3_GAZEBO__WALL_FOLLOWER_STATE_MACHINE_HPP_