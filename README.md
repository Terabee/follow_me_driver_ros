# ROS package for Terabee Follow-Me

This package is a ROS wrapper for `follow_me_driver` contained in [hardware_drivers](https://github.com/Terabee/hardware_drivers) package. It enables easy use of Terabee Follow-Me system in ROS environment.

## Supported hardware
This package works with Terabee Follow-Me system.

## Dependencies

This package depends on [hardware_drivers](https://github.com/Terabee/hardware_drivers). Please clone the repository into you workspace:

*  If you have ssh key setup for your github account:
```
cd ~/ros_ws/src
git clone git@github.com:Terabee/hardware_drivers.git
```
*  If you prefer to use https use this set of commands:
```
cd ~/ros_ws/src
git clone https://github.com/Terabee/hardware_drivers.git
```

## Building the packages
Clone the repository into your workspace:

*  If you have ssh key setup for your github account:
```
cd ~/ros_ws/src
git clone git@github.com:Terabee/follow_me_driver_ros.git
```
*  If you prefer to use https use this set of commands:
```
cd ~/ros_ws/src
git clone https://github.com/Terabee/follow_me_driver_ros.git
```
Navigate to your workspace and build:
```
cd ~/ros_ws
catkin build
source devel/setup.bash
```
## Nodes
### follow_me_master_beacon

With this node you can receive data (distance *in meters* and heading *in degrees*) from the system. It is intended to use with master beacon connected to the computer.

It provides following configuration options of the system:
*  switching between text and binary printout modes,
*  swapping beacons,
*  setting *Exponential Moving Average* filter number of samples (window size),
*  setting span between the beacons (in millimeters),
*  settings parameters for RS485 connection (Modbus slave id, baudrate, parity).

#### Running the node
After your workspace is built and sourced, you can run the node (set `_portname` to the actual port name where the master beacon is connected:
```
rosrun follow_me_driver_ros follow_me_master_beacon _portname:=/dev/ttyACM0
```
#### Subscribed topics
This node subscribes to the following topics:

*  `/follow_me_master_beacon/follow_me_autocalibrate` : publishing to this topic activates span autocalibration mode
*  `/follow_me_master_beacon/follow_me_config` : publishing to this topic sets parameters: printout mode, swap beacons, EMA filter window, span between the beacons
*  `/follow_me_master_beacon/follow_me_rs485_config` : publishing to this topic sets slave id, baudrate and parity of RS485 interface
*  `/follow_me_master_beacon/follow_me_test_cmd` : publishing to this topic triggers test command which returns actual configuration of the device

Example 1 of usage:
```
rostopic pub /follow_me_master_beacon/follow_me_config follow_me_driver_ros/FollowMeDriverConfig "printout_mode: 'Binary'
swap_beacons: true
ema_window: 10
beacons_span: 540"
```
Sets binary printout mode, swaps beacons, sets EMA filter to 10 samples and beacons span to 540 mm.

Example 2 of usage:
```
rostopic pub /follow_me_master_beacon/follow_me_rs485_config follow_me_driver_ros/FollowMeDriverRS485Config "rs485_slave_id: 3
rs485_baudrate: 19200
rs485_parity: 2"
```
Sets Modbus RTU slave id to 3, baud rate to 19200 and parity to 2 (Even).

To see list of valid values for each parameter, open respective \*.msg file.

#### Published topics
This node publishes to the topic:
*  `/follow_me_master_beacon/follow_me_polar_point_2d` : provides distance and heading of the remote control with respect to the beacons

### follow_me_remote_control

This node is intended to use with remote control connected to the computer.

It provides following configuration options of the remote control:
*  setting button operation mode (hold, toggle)
*  setting buzzer operation (enabled, disabled)

#### Running the node
After your workspace is built and sourced, you can run the node (set `_portname` to the actual port name where the master beacon is connected:
```
rosrun follow_me_driver_ros follow_me_remote_control _portname:=/dev/ttyUSB0
```
#### Subscribed topics
This node subscribes to the following topics:
*  `/follow_me_remote_control/follow_me_get_config` : publishing to this topic triggers command which returns actual configuration of the remote control
*  `/follow_me_remote_control/follow_me_set_config` : publishing to this topic sets remote control configuration (button mode and buzzer state)

Example of usage:
```
rostopic pub /follow_me_remote_control/follow_me_set_config follow_me_driver_ros/FollowMeRemoteControlConfig "button_mode: 'Hold'
buzzer_active: false"
```
Sets button mode to *Hold* and deactivates buzzer.

To see list of valid values for each parameter, open respective \*.msg file.

## Example

A basic subscriber is available in subdirectory `examples`.

In order to use the package in your own node, do the following:

In `CMakeLists.txt` add the package name `follow_me_driver_ros` to `find_package` and `catkin_package` commands.

In `package.xml` add:

```
<build_depend>follow_me_driver_ros</build_depend>
```
and
```
<exec_depend>follow_me_driver_ros</exec_depend>
```
