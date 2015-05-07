/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <miniking_ros/miniking_ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "miniking_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  MiniKingRos mkr(nh, nhp);
  ros::spin();
  return 0;
}
