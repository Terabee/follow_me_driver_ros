# ROS package for Terabee Follow-Me

This package is a wrapper for [hardware_drivers](https://github.com/Terabee/hardware_drivers) package enabling easy use of Terabee Follow-Me system in ROS environment.

## Supported hardware
This package works with Terabee Follow-Me system.

## Dependencies

This package depends on [hardware_drivers](https://github.com/Terabee/hardware_drivers), please clone the repository into you workspace or install in the system.

## Nodes
### follow_me_master_beacon node

With this node you can receive data (distance and heading from the system). It is intended to use with master beacon connected to the computer.

It provides following configuration options of the system:
*  switching between text and binary printout modes,
*  swapping beacons,
*  setting Exponential Moving Average filter window,
*  setting span between the beacons,
*  settings parameters for RS485 connection (slave id, baudrate, parity).  

This node subscribes to the following topics:

*  `/follow_me_master_beacon/follow_me_autocalibrate`: publishing to this topic activates span autocalibration mode
*  `/follow_me_master_beacon/follow_me_config`: publishing to this topic sets parameters: printout mode, swap beacons, EMA filter window, span between the beacons
*  `/follow_me_master_beacon/follow_me_rs485_config`: publishing to this topic sets slave id, baudrate and parity of RS485 interface
*  `/follow_me_master_beacon/follow_me_test_cmd`: publishing to this topic triggers test command which returns actual configuration of the device  

This node publishes to the topic:
*  `/follow_me_master_beacon/follow_me_polar_point_2d`: provides distance and heading of the remote control with respect to the beacons

### follow_me_remote_control node

This node is intended to use with remote control connected to the computer.

It provides following configuration options of the remote control:
*  setting button operation mode (hold, toggle)
*  setting buzzer operation (enabled, disabled)

This node subscribes to the following topics:
*  `/follow_me_remote_control/follow_me_get_config`: publishing to this topic triggers command which returns actual configuration of the remote control
*  `/follow_me_remote_control/follow_me_set_config`: publishing to this topic sets remote control configuration (button mode and buzzer state)

## Example

A basic subscriber is available in subdirectory `examples`.

In order to use the package in your own node, do the following:

In `CMakeLists.txt` add the package name `follow_me_driver_ros` to `find_package` and `catkin_package` commands.

In `package.xml` add:

`<build_depend>follow_me_driver_ros</build_depend>`

and

`<exec_depend>follow_me_driver_ros</exec_depend>`