#include "spinner_serial.h"


SpinnerSerial::SpinnerSerial(ros::NodeHandle& nh) :
  nh_(nh)
{
  // ROS private node
  ros::NodeHandle nh_private("~");

  // Debug mode
  bool debug;
  nh_private.param<bool>("debug", debug, false);

  if(debug){
    ROS_INFO("Setting output level to debug!");

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }


  // Get COM port parameter
  if (nh_private.getParam("com_port", com_port_))
  {
    ROS_DEBUG("Got 'com_port' parameter: %s", com_port_.c_str());
  }
  else
  {
    ROS_ERROR("Could not get com_port parameter.");
    ros::shutdown();
    return;
  }


  // Get twist_topic_name param
  std::string twist_topic_name;
  if (nh_private.getParam ("twist_topic_in", twist_topic_name))
  {
    ROS_DEBUG("Got 'twist_topic_in' parameter: %s", twist_topic_name.c_str());
  }
  else
  {
    ROS_ERROR("Could not get twist_topic_in parameter.");
    ros::shutdown();
    return;
  }



  // Set up serial com
  if(!setUpSerialCom())
  {
    ros::shutdown();
    return;
  }


  // Twist topic subscriber
  sub_velMsg_ = nh_.subscribe(twist_topic_name, 5, &SpinnerSerial::velCallback, this);


}

void SpinnerSerial::velCallback(const geometry_msgs::Twist::ConstPtr& msg) const
{
  static geometry_msgs::Twist twist_msg = *msg;

  const uint8_t vel_cmd = twist_msg->angular.z;

  ROS_DEBUG("Velocity command received = %u\n", vel_cmd);

  sendSerial(std::to_string(vel_cmd));
}

void SpinnerSerial::launch() const
{
  ros::Rate r(SPINNER_SERIAL_SPIN_RATE_);

  while(nh_.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  // Close COM port
  close(fd_);
}

bool SpinnerSerial::setUpSerialCom()
{

  // Set up the communication port
  struct termios term_attr;

  if  ((fd_ = open(com_port_.c_str(), O_WRONLY)) == -1){
    ROS_ERROR_STREAM("Terminal: Can’t open " << com_port_ << " port: " << strerror(errno));
    return false;
  }

  if (tcgetattr(fd_, &term_attr) != 0){
    ROS_ERROR("Terminal: tcgetattr() failed");
    return false;
  }

  // Set communication params
  term_attr.c_cflag = COM_SPEED_ | CS8 |  CLOCAL;
  term_attr.c_iflag = 0;
  term_attr.c_oflag = OPOST | ONLCR;
  term_attr.c_lflag = 0;

  if (tcsetattr(fd_, TCSAFLUSH, &term_attr) != 0){
    ROS_ERROR("Terminal: tcsetattr() failed");
    return false;
  }

  // Succesfull setup
  return true;

}

void SpinnerSerial::sendSerial(const std::string speed_cmd) const
{
  ROS_DEBUG_STREAM("Sending speed command = " << speed_cmd << "\n");

  const std::string datastream = (char)CMD_BYTE_ + speed_cmd;

  ROS_DEBUG_STREAM("DataStream = " << datastream << "\n" << datastream.length());

  const ssize_t sentBytes = write(fd_, datastream.c_str(), datastream.length());
  // const ssize_t sentBytes = write(fd_, speed_cmd.c_str(), (speed_cmd.c_str()).length;

  if( sentBytes == -1 )
  {
    ROS_ERROR_STREAM ("sendSerial unsuccesful: " << strerror(errno));
  }

}
