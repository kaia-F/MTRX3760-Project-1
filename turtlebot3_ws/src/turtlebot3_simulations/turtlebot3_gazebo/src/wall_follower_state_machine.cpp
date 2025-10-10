#include "turtlebot3_gazebo/wall_follower_state_machine.hpp"

//Constructor - initializes the state machine to forward driving state
WallFollowerStateMachine::WallFollowerStateMachine()
: current_state_(TB3_DRIVE_FORWARD),
  previous_state_(GET_TB3_DIRECTION),
  forward_start_x_(0.0),
  forward_start_y_(0.0)
{
}

/**
 * @brief Main update function - this is the "brain" of the wall follower
 * 
 * This function contains all the decision-making logic extracted from
 * the original update_callback().
 */
VelocityCommand WallFollowerStateMachine::update(
  const SensorData& sensors,
  const RobotState& robot,
  double (*normalise_angle)(double))
{
  // Get sensor readings
  double front = sensors.get_front();
  double left = sensors.get_left();
  double right = sensors.get_right();
  
  // Calculate wall detection flags
  bool front_clear = (front > WallFollowerConfig::FRONT_OBSTACLE_DISTANCE);
  bool wall_on_right = (right < WallFollowerConfig::WALL_DETECTION_RANGE);
  bool wall_on_left = (left < WallFollowerConfig::WALL_DETECTION_RANGE);
  
  // Update previous state for change detection
  previous_state_ = current_state_;
  
  // Execute current state logic
  switch (current_state_)
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
        current_state_ = TB3_RIGHT_TURN_180;
      }
      else if (!wall_on_right)
      {
        // Lost the wall, turn right to find it
        forward_start_x_ = robot.get_x();
        forward_start_y_ = robot.get_y();
        current_state_ = TB3_PRE_RIGHT_COMMIT;
      }
      else if (!front_clear)
      {
        // Obstacle ahead, turn left
        current_state_ = TB3_LEFT_TURN;
      }
      else
      {
        // Follow the wall
        current_state_ = TB3_DRIVE_FORWARD;
      }
      return VelocityCommand(0.0, 0.0);  // Stop during decision
    }

    case TB3_DRIVE_FORWARD:
    {
      // SMOOTH wall following with proportional control
      double linear = WallFollowerConfig::LINEAR_VELOCITY;
      double angular = 0.0;
      
      if (wall_on_right)
      {
        // Calculate error: positive = too far, negative = too close
        double error = right - WallFollowerConfig::TARGET_WALL_DISTANCE;
        
        // Deadband: Ignore small errors to prevent oscillation
        if (std::fabs(error) > WallFollowerConfig::CONTROL_DEADBAND)
        {
          // Proportional control: steer toward wall if too far, away if too close
          angular = -WallFollowerConfig::PROPORTIONAL_GAIN * error;
          
          // Limit angular velocity to prevent oscillation
          if (angular > WallFollowerConfig::MAX_ANGULAR_VELOCITY) 
            angular = WallFollowerConfig::MAX_ANGULAR_VELOCITY;
          if (angular < -WallFollowerConfig::MAX_ANGULAR_VELOCITY) 
            angular = -WallFollowerConfig::MAX_ANGULAR_VELOCITY;
          
          // Reduce linear speed during corrections
          if (std::fabs(angular) > WallFollowerConfig::CORRECTION_THRESHOLD)
          {
            linear = WallFollowerConfig::LINEAR_VELOCITY * 
                     WallFollowerConfig::CORRECTION_SPEED_FACTOR;
          }
        }
      }
      
      // Check if conditions changed
      if (!front_clear || !wall_on_right)
      {
        current_state_ = GET_TB3_DIRECTION;
      }
      
      return VelocityCommand(linear, angular);
    }

    case TB3_RIGHT_TURN:
    {
      // Turn 90° clockwise (negative angular velocity)
      double angle_diff = robot.get_angle_turned(normalise_angle);
      
      if (std::fabs(angle_diff) >= (WallFollowerConfig::TURN_ANGLE_90 - 
                                    WallFollowerConfig::ANGLE_TOLERANCE))
      {
        // Turn complete - record position for next commit phase
        forward_start_x_ = robot.get_x();
        forward_start_y_ = robot.get_y();
        current_state_ = TB3_POST_RIGHT_COMMIT;
        return VelocityCommand(0.0, 0.0);  // Stop
      }
      else
      {
        return VelocityCommand(0.0, -WallFollowerConfig::ANGULAR_VELOCITY);
      }
    }

    case TB3_LEFT_TURN:
    {
      // Turn 90° counter-clockwise (positive angular velocity)
      double angle_diff = robot.get_angle_turned(normalise_angle);
      
      if (std::fabs(angle_diff) >= (WallFollowerConfig::TURN_ANGLE_90 - 
                                    WallFollowerConfig::ANGLE_TOLERANCE))
      {
        // Turn complete - return to decision state
        current_state_ = GET_TB3_DIRECTION;
        return VelocityCommand(0.0, 0.0);  // Stop
      }
      else
      {
        return VelocityCommand(0.0, WallFollowerConfig::ANGULAR_VELOCITY);
      }
    }

    case TB3_RIGHT_TURN_180:
    {
      // Turn 180° (for special case at startup)
      double angle_diff = robot.get_angle_turned(normalise_angle);

      if (std::fabs(angle_diff) >= (WallFollowerConfig::TURN_ANGLE_180 - 
                                    WallFollowerConfig::ANGLE_TOLERANCE))
      {
        current_state_ = TB3_DRIVE_FORWARD;
        return VelocityCommand(0.0, 0.0);  // Stop
      }
      else
      {
        return VelocityCommand(0.0, WallFollowerConfig::ANGULAR_VELOCITY);
      }
    }

    case TB3_PRE_RIGHT_COMMIT:
    {
      // Drive forward a bit before turning right
      double distance = robot.distance_to(forward_start_x_, forward_start_y_);

      if (distance >= WallFollowerConfig::PRE_TURN_COMMIT_DISTANCE)
      {
        // Moved far enough - now turn right
        current_state_ = TB3_RIGHT_TURN;
        return VelocityCommand(0.0, 0.0);  // Stop before turn
      }
      else
      {
        return VelocityCommand(WallFollowerConfig::LINEAR_VELOCITY, 0.0);
      }
    }

    case TB3_POST_RIGHT_COMMIT:
    {
      // Drive forward a bit after turning right
      double distance = robot.distance_to(forward_start_x_, forward_start_y_);

      if (distance >= WallFollowerConfig::POST_TURN_COMMIT_DISTANCE)
      {
        // Moved far enough - return to decision
        current_state_ = GET_TB3_DIRECTION;
        return VelocityCommand(0.0, 0.0);  // Stop
      }
      else
      {
        return VelocityCommand(WallFollowerConfig::LINEAR_VELOCITY, 0.0);
      }
    }

    default:
      // Unknown state - reset to decision
      current_state_ = GET_TB3_DIRECTION;
      return VelocityCommand(0.0, 0.0);
  }
}