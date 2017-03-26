#include "spinner_serial.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spinner_serial_node");
  ros::NodeHandle nh;

  SpinnerSerial spinnerSerial(nh);

  spinnerSerial.launch();

  ROS_INFO("Running SpinnerSerial node");
}
