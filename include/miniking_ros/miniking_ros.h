/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef MINIKING_ROS_H
#define MINIKING_ROS_H

#include <libminiking/MiniKing.h>

#include <ros/ros.h>
#include <miniking_ros/MiniKingConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include <string>


class MiniKingRos {
 public:
  MiniKingRos(ros::NodeHandle nh, ros::NodeHandle nhp);
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher pub_;
  MiniKing* mk_;
  std::string port_;
  SonarType type_;            // Sonar Type
  Frequency frequency_;      // Channel Frequency
  Resolution resolution_;    // Step Angle Size
  typedef miniking_ros::MiniKingConfig DynConfig;
  DynConfig config_;
  typedef dynamic_reconfigure::Server<DynConfig> ReconfigureServer;
  ReconfigureServer reconfigure_server_;
  boost::mutex config_mutex_;
  ros::Timer timer_;
  bool first_config_;

  void updateConfig(DynConfig& config, uint32_t level);
  int getFrequency(const std::string& s);
  int getType(const std::string& s);
  int getResolution(const std::string& s);
  float getResolutionValue(const int r);
  void timerCallback(const ros::TimerEvent& event);
  void printConfigurations(void);
};
#endif  // MINIKING_ROS_H
