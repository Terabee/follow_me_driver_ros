#include <ros/ros.h>
#include "follow_me_driver_ros/PolarPoint2D.h"

void chatterCallback(const follow_me_driver_ros::PolarPoint2DConstPtr& point)
{
  ROS_INFO_STREAM("Distance: " << point->distance << "\tHeading: " << point->heading);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_follow_me_subscriber");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("follow_me_master_beacon/follow_me_polar_point_2d", 1, chatterCallback);

  ros::spin();

  return 0;
}
