#ifndef TURTLEBOT3_GAZEBO_UTILS_HPP_
#define TURTLEBOT3_GAZEBO_UTILS_HPP_

#include <cmath>

inline double normalise_angle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

#endif  // TURTLEBOT3_GAZEBO_UTILS_HPP_
