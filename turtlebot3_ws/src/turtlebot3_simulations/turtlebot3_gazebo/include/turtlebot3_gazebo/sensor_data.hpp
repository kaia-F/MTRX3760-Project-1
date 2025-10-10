#ifndef TURTLEBOT3_GAZEBO__SENSOR_DATA_HPP_
#define TURTLEBOT3_GAZEBO__SENSOR_DATA_HPP_

/**
 * @file sensor_data.hpp
 * @brief Encapsulates LIDAR sensor readings from three directions
 * 
 * This class wraps the scan_data_ array to provide meaningful
 * access to front, left, and right distance measurements.
 */

// Define sensor array indices as constants
#define CENTER 0
#define LEFT   1
#define RIGHT  2

class SensorData
{
public:
  //Default constructor - initializes all readings to zero
  SensorData()
  {
    scan_data_[CENTER] = 0.0;
    scan_data_[LEFT] = 0.0;
    scan_data_[RIGHT] = 0.0;
  }
  
  //Get front distance reading, return distance to obstacle in front
  double get_front() const
  {
    return scan_data_[CENTER];
  }
  
  //Get left distance reading
  double get_left() const
  {
    return scan_data_[LEFT];
  }
  
  //Get right distance reading
  double get_right() const
  {
    return scan_data_[RIGHT];
  }
  
  //Set front distance reading
  void set_front(double distance)
  {
    scan_data_[CENTER] = distance;
  }
  
  //Set left distance reading
  void set_left(double distance)
  {
    scan_data_[LEFT] = distance;
  }
  
  //Set right distance reading
  void set_right(double distance)
  {
    scan_data_[RIGHT] = distance;
  }
  
  //Check if sensor data has been initialized
  bool is_ready() const
  {
    return (scan_data_[CENTER] != 0.0 || 
            scan_data_[LEFT] != 0.0 || 
            scan_data_[RIGHT] != 0.0);
  }

private:
  // Store the three LIDAR readings (front, left, right)
  double scan_data_[3];
};

#endif  // TURTLEBOT3_GAZEBO__SENSOR_DATA_HPP_