#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "follow_me_driver/follow_me_driver.hpp"
#include "follow_me_driver_ros/FollowMeRemoteControlConfig.h"
#include "serial_communication/serial.hpp"

namespace terabee {

class FollowMeRemoteControlROS
{
public:
  FollowMeRemoteControlROS(ros::NodeHandle& nh,
                           std::shared_ptr<serial_communication::ISerial> serialIf);

  void spin();
  void start() { is_started_ = true; }
  void stop() { is_started_ = false; }

private:
  FollowMeRemoteControl remote_control_;
  ros::NodeHandle& nodeHandle_;
  bool is_started_ = false;
  double time_step_;

  std::string portname_;
  int baudrate_;
  int serial_timeout_ms_;
  const serial_communication::ISerial::parity_t parity_ = serial_communication::ISerial::parity_none;
  const serial_communication::ISerial::bytesize_t data_bits_ = serial_communication::ISerial::eightbits;
  const serial_communication::ISerial::stopbits_t stop_bits_ = serial_communication::ISerial::stopbits_one;

  std::unique_ptr<ros::Rate> rate_;

  ros::Subscriber follow_me_set_remote_config_sub_;
  ros::Subscriber follow_me_get_remote_config_sub_;

  void updateParametersCallback(const follow_me_driver_ros::FollowMeRemoteControlConfig &config);
  void getParametersCallback(std_msgs::Empty msg) const;
};

FollowMeRemoteControlROS::FollowMeRemoteControlROS(ros::NodeHandle &nh,
                                                   std::shared_ptr<serial_communication::ISerial> serialIf):
  remote_control_(serialIf),
  nodeHandle_(nh)
{
  nodeHandle_.param("time_step", time_step_, 0.01);
  nodeHandle_.param<std::string>("portname", portname_, "/dev/ttyUSB0");
  nodeHandle_.param("baudrate", baudrate_, 115200);
  nodeHandle_.param("serial_timeout_ms", serial_timeout_ms_, 800);

  serialIf->setPortName(portname_);
  serialIf->setBaudrate(baudrate_);
  serialIf->setTimeout(std::chrono::milliseconds(serial_timeout_ms_));

  serialIf->open();

  if(serialIf->isOpen()){
    ROS_INFO_STREAM("Serial port open: " << portname_.c_str());
  }
  else
  {
    ROS_ERROR_STREAM("Could not open: " << portname_.c_str());
    ros::shutdown();
  }

  rate_ = std::make_unique<ros::Rate>(1.0/time_step_);

  follow_me_set_remote_config_sub_ =
      nodeHandle_.subscribe("follow_me_set_config", 1,
                            &FollowMeRemoteControlROS::updateParametersCallback, this);

  follow_me_get_remote_config_sub_ =
      nodeHandle_.subscribe("follow_me_get_config", 1,
                            &FollowMeRemoteControlROS::getParametersCallback, this);

  ROS_INFO("follow_me_driver_ros launched successfully.");
}

void FollowMeRemoteControlROS::spin()
{
  while (ros::ok() && is_started_)
  {
    ros::spinOnce();
    rate_->sleep();
  }
}

void FollowMeRemoteControlROS::updateParametersCallback(const follow_me_driver_ros::FollowMeRemoteControlConfig &config)
{
  if (config.button_mode == "Toggle")
  {
    remote_control_.setButtonMode(FollowMeRemoteControl::button_mode::toggle);
  }
  else if (config.button_mode == "Hold")
  {
    remote_control_.setButtonMode(FollowMeRemoteControl::button_mode::hold);
  }
  else
  {
    ROS_WARN("Unknown button mode");
  }
  remote_control_.setBuzzer(config.buzzer_active);
}

void FollowMeRemoteControlROS::getParametersCallback(std_msgs::Empty msg) const
{
  FollowMeRemoteControl::button_mode btn_mode;
  bool buzzer_activated;
  if (remote_control_.retrieveRemoteParameters(btn_mode, buzzer_activated))
  {
    ROS_INFO_STREAM("Buzzer " << (buzzer_activated ? "activated" : "deactivated") <<
                    "; Button " << (btn_mode == FollowMeRemoteControl::button_mode::toggle ? "toggle" : "hold") << " mode"
                    );
  }
}

}  // namespace terabee

int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_me_remote_control");
  ros::NodeHandle nh("~");

  std::shared_ptr<terabee::serial_communication::ISerial> serialInterface =
      std::make_shared<terabee::serial_communication::Serial>("/dev/ttyUSB0");

  terabee::FollowMeRemoteControlROS followMeDriver(nh, serialInterface);

  followMeDriver.start();
  followMeDriver.spin();
}
