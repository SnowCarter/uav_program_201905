#include <ros/ros.h>
#include "filter_kalman.h"
#include <pcl/filters/random_sample.h>


int main(int argc, char **argv)
{
    ROS_INFO("i'm in1");
  ros::init(argc,argv,"filter_kalman");
  ros::NodeHandle node("~");
  filter_kalman::Center_map center_map(node);
  center_map.initializer(node);

  ros::spin();
  return 0;



}

