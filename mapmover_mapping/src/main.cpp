#include <ros/ros.h>

#include "MapmoverMappingRos.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mapmover_slam");

  MapmoverMappingRos sm;

  ros::spin();

  return(0);
}

