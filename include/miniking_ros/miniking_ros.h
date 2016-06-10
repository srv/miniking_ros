/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef MINIKING_ROS_H
#define MINIKING_ROS_H

#include <libminiking/MiniKing.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <miniking_ros/MiniKingConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include <string>
#include <std_srvs/Empty.h>


class MiniKingRos {
 public:
  MiniKingRos(ros::NodeHandle nh, ros::NodeHandle nhp);
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher pub_;
  MiniKing* mk_;
  std::string port_;
  bool sea_operation_;
  double min_depth_;
  double depth_;
  ros::Time depth_stamp_;
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
  ros::Subscriber depth_sub_;
  ros::ServiceClient sonar_on_;

  void updateConfig(DynConfig& config, uint32_t level);
  int getFrequency(const std::string& s);
  const char* getFrequencyChar(const int f);
  int getType(const std::string& s);
  const char* getTypeChar(const int t);
  int getResolution(const std::string& s);
  float getResolutionValue(const int r);
  const char* getResolutionChar(const int r);
  void depthCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void timerCallback(const ros::TimerEvent& event);
  void printConfigurations(void);
};
#endif  // MINIKING_ROS_H
