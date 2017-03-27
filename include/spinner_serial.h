#ifndef SPINNER_SERIAL_H
#define SPINNER_SERIAL_H

#include "ros/ros.h"
#include <boost/asio.hpp>
#include "geometry_msgs/Twist.h"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/exceptions.h>

class SpinnerSerial
{
public:
  /*
   * Class constructor
   */
  SpinnerSerial(ros::NodeHandle& nh);

  /*
   * Class destructor
   */
  ~SpinnerSerial() = default;

  /*
   * Callback for twist msgs
   */
  void velCallback(const geometry_msgs::Twist::ConstPtr& msg) const;

  /*
   * Main function for checking and callbacks subscribing. Closes port when finished
   */
  void launch() const;

  /*
   * Set up serial port
   */
  bool setUpSerialCom();

  /*
   * Sending commands to serial
   */
  virtual void sendSerial(const std::string speed_cmd) const;

private:
  // ROS members
  ros::NodeHandle& nh_;
  ros::Subscriber sub_velMsg_;

  // Storage for last Twist
  static geometry_msgs::Twist last_twist_;

  // COM members
  int fd_;
  std::string com_port_;

  // Static members for ROS
  static constexpr uint8_t SPINNER_SERIAL_SPIN_RATE_ = 50U;

  // Static members for serial com
  static constexpr uint8_t N_BYTES_ = 2U;
  static constexpr uint8_t CMD_BYTE_ = 0x53U;
  static constexpr speed_t COM_SPEED_ = B9600;




};

#endif // SPINNER_SERIAL_H
