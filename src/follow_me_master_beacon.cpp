#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "follow_me_driver/follow_me_driver.hpp"
#include "follow_me_driver_ros/PolarPoint2D.h"
#include "follow_me_driver_ros/FollowMeDriverConfig.h"
#include "follow_me_driver_ros/FollowMeDriverRS485Config.h"
#include "serial_communication/serial.hpp"

namespace terabee {

class FollowMeMasterBeaconROS
{
public:
  FollowMeMasterBeaconROS(ros::NodeHandle& nh,
                    std::shared_ptr<serial_communication::ISerial> serialIf);

  void spin();
  void start() { is_started_ = true; }
  void stop() { is_started_ = false; }

private:
  FollowMeMasterBeacon master_beacon_;
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

  ros::Publisher point_publisher_;
  ros::Subscriber follow_me_config_sub_;
  ros::Subscriber follow_me_rs485_config_sub_;
  ros::Subscriber follow_me_autocalibrate_sub_;
  ros::Subscriber follow_me_test_cmd_;

  void updateParametersCallback(const follow_me_driver_ros::FollowMeDriverConfig &config);
  void updateParametersRS485Callback(const follow_me_driver_ros::FollowMeDriverRS485Config &config);
  void autoCalibrateCallback(std_msgs::Empty msg) const;
  void testCommandCallback(std_msgs::Empty msg) const;
  bool processTestCommand(std::string test_result) const;
};

FollowMeMasterBeaconROS::FollowMeMasterBeaconROS(ros::NodeHandle& nh,
                                     std::shared_ptr<serial_communication::ISerial> serialIf):
  master_beacon_(serialIf),
  nodeHandle_(nh)
{
  nodeHandle_.param("time_step", time_step_, 0.01);
  nodeHandle_.param<std::string>("portname", portname_, "/dev/ttyACM0");
  nodeHandle_.param("baudrate", baudrate_, 115200);
  nodeHandle_.param("serial_timeout_ms", serial_timeout_ms_, 200);

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
  master_beacon_.printoutModeBin();

  rate_ = std::make_unique<ros::Rate>(1.0/time_step_);

  point_publisher_ = nodeHandle_.advertise<follow_me_driver_ros::PolarPoint2D>("follow_me_polar_point_2d", 1);
  follow_me_config_sub_ =
      nodeHandle_.subscribe("follow_me_config", 1,
                            &FollowMeMasterBeaconROS::updateParametersCallback, this);

  follow_me_rs485_config_sub_ =
      nodeHandle_.subscribe("follow_me_rs485_config", 1,
                            &FollowMeMasterBeaconROS::updateParametersRS485Callback, this);

  follow_me_autocalibrate_sub_ =
      nodeHandle_.subscribe("follow_me_autocalibrate", 1,
                            &FollowMeMasterBeaconROS::autoCalibrateCallback, this);

  follow_me_test_cmd_ =
      nodeHandle_.subscribe("follow_me_test_cmd", 1,
                            &FollowMeMasterBeaconROS::testCommandCallback, this);

  ROS_INFO("follow_me_driver_ros launched successfully.");
}

void FollowMeMasterBeaconROS::spin()
{
  while(ros::ok() && is_started_)
  {
    PolarPoint2D recv_point;
    if (master_beacon_.process(recv_point))
    {
      follow_me_driver_ros::PolarPoint2D point;
      point.header.stamp = ros::Time::now();
      point.distance = recv_point.distance;
      point.heading = recv_point.heading;
      point_publisher_.publish(point);
    }

    ros::spinOnce();
    rate_->sleep();
  }
}

void FollowMeMasterBeaconROS::autoCalibrateCallback(std_msgs::Empty msg) const
{
  master_beacon_.spanAutoCalibrate();
}

void FollowMeMasterBeaconROS::testCommandCallback(std_msgs::Empty msg) const
{
  if(!processTestCommand(master_beacon_.testCommand()))
  {
    ROS_WARN("Error while processing test command");
  }
}

bool FollowMeMasterBeaconROS::processTestCommand(std::string test_result) const
{
  if (test_result.empty() || test_result.find("ID TB-FM") == std::string::npos)
  {
    ROS_INFO("Failed to retrieve parameters from master beacon");
    return false;
  }
  size_t rs485_slave_id_pos = test_result.find("RS485@ ") + 7;
  size_t rs485_baud_pos = test_result.find(" ", rs485_slave_id_pos) + 1;
  size_t rs485_parity_pos = test_result.find(" ", rs485_baud_pos) + 1;
  size_t rs485_parity_pos_end = test_result.find(" UWB", rs485_parity_pos);
  size_t span_pos = test_result.find("D:", rs485_parity_pos_end) + 2;
  size_t ema_window_pos = test_result.find(":", span_pos) + 1;
  size_t swap_beacons_pos = test_result.find(":", ema_window_pos) + 1;
  size_t print_out_mode_pos = test_result.find(":", swap_beacons_pos) + 1;

  int rs485_slave_id = std::stoi(test_result.substr(rs485_slave_id_pos, rs485_baud_pos - rs485_slave_id_pos - 1));
  int span = std::stoi(test_result.substr(span_pos, ema_window_pos - span_pos - 1));
  int ema_window = std::stoi(test_result.substr(ema_window_pos, swap_beacons_pos - ema_window_pos - 1));
  bool swap_beacons = std::stoi(test_result.substr(swap_beacons_pos).c_str());
  FollowMeMasterBeacon::printout_mode printout_mode =
      static_cast<FollowMeMasterBeacon::printout_mode>(std::stoi(test_result.substr(print_out_mode_pos).c_str()));
  FollowMeMasterBeacon::rs485_parity rs485_parity =
      static_cast<FollowMeMasterBeacon::rs485_parity>(std::stoi(test_result.substr(rs485_parity_pos, rs485_parity_pos_end - rs485_parity_pos)));
  int rs485_baudrate = std::stoi(test_result.substr(rs485_baud_pos, rs485_parity_pos - rs485_baud_pos - 1));

  std::string parity_str;
  switch (rs485_parity)
  {
  case FollowMeMasterBeacon::rs485_parity_none:
    parity_str = "None";
    break;
  case FollowMeMasterBeacon::rs485_parity_odd:
    parity_str = "Odd";
    break;
  case FollowMeMasterBeacon::rs485_parity_even:
    parity_str = "Even";
    break;
  }
  ROS_INFO_STREAM("Parameters:"
                  "\n\tRS485 slave id:\t" << rs485_slave_id <<
                  "\n\tRS485 baudrate:\t" << rs485_baudrate <<
                  "\n\tRS485 parity:\t" << parity_str <<
                  "\n\tPrintout mode:\t" << (printout_mode == FollowMeMasterBeacon::text ? "Text" : "Binary") <<
                  "\n\tBeacons span:\t" << span <<
                  "\n\tSwap beacons:\t" << (swap_beacons ? "Enabled" : "Disabled") <<
                  "\n\tEMA window:\t" << ema_window);
  return true;
}

void FollowMeMasterBeaconROS::updateParametersCallback(const follow_me_driver_ros::FollowMeDriverConfig &config)
{
  if (!master_beacon_.setEMAWindow(config.ema_window))
  {
    ROS_WARN("Failed to set ema_window parameter!");
  }
  if (!master_beacon_.setBeaconsSpan(config.beacons_span))
  {
    ROS_WARN("Failed to set beacons_span parameter!");
  }
  if (!master_beacon_.swapBeacons(config.swap_beacons))
  {
    ROS_WARN("Failed to set swap_beacons parameter!");
  }

  if (config.printout_mode == "Binary")
  {
    if (!master_beacon_.printoutModeBin())
    {
      ROS_WARN("Failed to set printout_mode: Binary!");
    }
  }
  else if (config.printout_mode == "Text")
  {
    if (!master_beacon_.printoutModeText())
    {
      ROS_WARN("Failed to set printout_mode: Text!");
    }
  }
  else
  {
    ROS_WARN("Unknown printout mode");
  }
}

void FollowMeMasterBeaconROS::updateParametersRS485Callback(const follow_me_driver_ros::FollowMeDriverRS485Config &config)
{
  if (!master_beacon_.setRS485_Parameters(config.rs485_slave_id, config.rs485_baudrate, static_cast<terabee::FollowMeMasterBeacon::rs485_parity>(config.rs485_parity)))
  {
    ROS_WARN("Failed to set RS485 parameters");
  }
}

}  // namespace terabee

int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_me_master_beacon");
  ros::NodeHandle nh("~");

  std::shared_ptr<terabee::serial_communication::ISerial> serialInterface =
      std::make_shared<terabee::serial_communication::Serial>("/dev/ttyACM0");

  terabee::FollowMeMasterBeaconROS followMeDriver(nh, serialInterface);

  followMeDriver.start();
  followMeDriver.spin();
  return 0;
}
