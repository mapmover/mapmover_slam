#include <ros/ros.h>

#include "MapmoverMapperRos.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mapmover_slam");

  MapmoverMapperRos sm;

  ros::spin();

  return(0);
}

