#ifndef TURTLEBOT3_GAZEBO_UTILS_HPP_
#define TURTLEBOT3_GAZEBO_UTILS_HPP_

#include <cmath>

inline double normalise_angle(double angle)
{
  double remainder = std::fmod(angle, 2.0 *M_PI);

  if (remainder > M_PI)
  {
    remainder -= 2.0 * M_PI;
  }
  else if (remainder < -M_PI)
  {
    remainder += 2.0 * M_PI;
  }
  return remainder;

  /*
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
  */
}

#endif  // TURTLEBOT3_GAZEBO_UTILS_HPP_
