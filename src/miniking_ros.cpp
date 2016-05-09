/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <miniking_ros/miniking_ros.h>

#include <miniking_ros/AcousticBeam.h>

#include <boost/function.hpp>

#include <string>

#define Ksonar_type 3
#define Kresolution 4
#define Kfrequency 13
static const char* sonar_type_char[] = {"Imaging", "Sidescan", "Profiling"};
static const char* resolution_char[] = {"Low", "Medium", "High", "Ultimate"};
static const char* frequency_char[] = {"f0", "f325", "f580", "f675", "f795", "f935", "f1210", "f200", "f1700", "f2000", "f500", "f1500", "f295"};
static int sonar_type_int[] = {2, 3, 5};
static int frequency_int[] = {0, 325, 580, 675, 795, 935, 1210, 200, 1700, 2000, 500, 1500, 295};
static int resolution_int[] = {32, 16, 8,  4};
static float resolution_value[] = {1.8, 0.9, 0.45, 0.225};

/**
 * @brief MiniKingRos constructor
 *
 * @param nh Public node handle
 * @param nhp Private node handle
 */
MiniKingRos::MiniKingRos(ros::NodeHandle nh, ros::NodeHandle nhp)
    : nh_(nh), nhp_(nhp), first_config_(true) {
  // Get the MiniKing port
  nhp_.param("port", port_, std::string("/dev/ttyUSB2"));

  // Get operational variables
  nhp_.param("sea_operation", sea_operation_, false);
  nhp_.param("min_depth", min_depth_, 0.5);

  // Init sonar
  mk_ = new MiniKing(const_cast<char*>(port_.c_str()), 0);
  mk_->initSonar();

  // Start dynamic_reconfigure & run configure()
  dynamic_reconfigure::Server<DynConfig>::CallbackType f;
  f = boost::bind(&MiniKingRos::updateConfig, this, _1, _2);
  reconfigure_server_.setCallback(f);

  // Setup subscribers
  if (sea_operation_)
  {
    // Wait for arduino service
    sonar_on_ = nh_.serviceClient<std_srvs::Empty>("/sonar_on");
    while (!sonar_on_.waitForExistence()) {
      ROS_INFO_STREAM_ONCE("[MiniKing]: Waiting for /sonar_on service to be available.");
    }

    // Enable sonar
    ros::Duration(2.0).sleep();
    ROS_INFO_STREAM("[MiniKing]: Calling /sonar_on service...");
    std_srvs::Empty srv;
    sonar_on_.call(srv);

    depth_ = -1.0;
    depth_sub_ = nh_.subscribe("depth", 1, &MiniKingRos::depthCb, this);
  }

  // Setup publishers
  pub_ = nhp_.advertise<miniking_ros::AcousticBeam>("sonar", 1);

  // Lastly, setup timer callback
  timer_ = nhp.createTimer(ros::Duration(0.01),
    &MiniKingRos::timerCallback, this);
}

void MiniKingRos::printConfigurations(void) {
  DynConfig config;
  config.resolution = mk_->getResolution();
  config.continuous = mk_->getContinuous();
  config.inverted = mk_->getInverted();
  config.stare = mk_->getStare();
  config.disable_motor = mk_->getMotorDisabled();
  config.disable_trans = mk_->getTransmitDisabled();
  config.type = mk_->getSonarType();
  config.frequency = mk_->getFrequency();
  config.range = mk_->getRange();
  config.left_limit = mk_->getLeftLim();
  config.right_limit = mk_->getRightLim();
  config.gain = mk_->getGain();
  config.bins = mk_->getBins();

  ROS_INFO_STREAM("[MiniKing]: Configuration" <<
    "\n\t* Resolution:   " << getResolutionChar(config.resolution) <<
    "\n\t* Continuous:   " << config.continuous <<
    "\n\t* Inverted:     " << config.inverted <<
    "\n\t* Stare:        " << config.stare <<
    "\n\t* DisableMotor: " << config.disable_motor <<
    "\n\t* SonarType:    " << getTypeChar(config.type) <<
    "\n\t* Frequency:    " << getFrequencyChar(config.frequency) <<
    "\n\t* Range:        " << config.range <<
    "\n\t* LeftLimit:    " << config.left_limit <<
    "\n\t* RightLimit:   " << config.right_limit <<
    "\n\t* Gain:         " << config.gain <<
    "\n\t* Bins:         " << config.bins);
}

void MiniKingRos::depthCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  depth_ = msg->pose.pose.position.z;
  depth_stamp_ = msg->header.stamp;
}

void MiniKingRos::timerCallback(const ros::TimerEvent& event) {

  // In operation mode?
  if (sea_operation_) {
    if (depth_ < 0.0) {
      ROS_WARN_STREAM("[MiniKing]: Invalid depth (" << depth_ << ").");
      return;
    }

    ros::Duration d = ros::Time::now() - depth_stamp_;
    if (d > ros::Duration(2.0)) {
      ROS_WARN_STREAM("[MiniKing]: No depth samples since " << d.toSec() << "s ago.");
      return;
    }

    if (depth_ < min_depth_)
      return;
  }

  boost::mutex::scoped_lock lock(config_mutex_);
  miniking_ros::AcousticBeam beam;
  beam.header.frame_id = "sonar";
  beam.header.stamp = ros::Time::now();
  beam.range_max = config_.range;
  beam.angle_step = getResolutionValue(config_.resolution);
  beam.left_limit = config_.left_limit;
  beam.right_limit = config_.right_limit;
  beam.bins = config_.bins;
  beam.angle = mk_->getPosition();
  unsigned char *data = mk_->getScanLine();
  for (int i = 0; i < mk_->getDataLength(); i++)
    beam.intensities.push_back(data[i]);
  pub_.publish(beam);
}

void MiniKingRos::updateConfig(DynConfig& config, uint32_t level) {
  boost::mutex::scoped_lock lock(config_mutex_);

  bool needs_update = false;

  if (config.resolution != config_.resolution || first_config_) {
    needs_update = true;
    config_.resolution = config.resolution;
    mk_->setResolution(static_cast<Resolution>(config.resolution));
    ROS_INFO_STREAM("[MiniKing]: Resolution set to " << config.resolution);
  }
  if (config.continuous != config_.continuous || first_config_) {
    needs_update = true;
    config_.continuous = config.continuous;
    mk_->setContinuous(config.continuous);
    ROS_INFO_STREAM("[MiniKing]: Continuous set to " << config.continuous);
  }
  if (config.inverted != config_.inverted || first_config_) {
    needs_update = true;
    config_.inverted = config.inverted;
    mk_->setInverted(config.inverted);
    ROS_INFO_STREAM("[MiniKing]: Inverted set to " << config.inverted);
  }
  if (config.stare != config_.stare || first_config_) {
    needs_update = true;
    config_.stare = config.stare;
    mk_->setStare(config.stare);
    ROS_INFO_STREAM("[MiniKing]: Stare set to " << config.stare);
  }
  if (config.disable_motor != config_.disable_motor || first_config_) {
    needs_update = true;
    config_.disable_motor = config.disable_motor;
    mk_->setMotorDisabled(config.disable_motor);
    ROS_INFO_STREAM("[MiniKing]: DisableMotor set to " << config.disable_motor);
  }
  if (config.disable_trans != config_.disable_trans || first_config_) {
    needs_update = true;
    config_.disable_trans = config.disable_trans;
    mk_->setTransmitDisabled(config.disable_trans);
    ROS_INFO_STREAM("[MiniKing]: Resolution set to " << config.resolution);
  }
  if (config.type != config_.type || first_config_) {
    needs_update = true;
    config_.type = config.type;
    mk_->setSonarType(static_cast<SonarType>(config.type));
    ROS_INFO_STREAM("[MiniKing]: SonarType set to " << config.type);
  }
  if (config.frequency != config_.frequency || first_config_) {
    needs_update = true;
    config_.frequency = config.frequency;
    mk_->setFrequency(static_cast<Frequency>(config.frequency));
    ROS_INFO_STREAM("[MiniKing]: Frequency set to " << config.frequency);
  }
  if (config.range != config_.range || first_config_) {
    needs_update = true;
    config_.range = config.range;
    mk_->setRange(config.range);
    ROS_INFO_STREAM("[MiniKing]: Range set to " << config.range);
  }
  if (config.left_limit != config_.left_limit || first_config_) {
    needs_update = true;
    config_.left_limit = config.left_limit;
    mk_->setLeftLim(config.left_limit);
    ROS_INFO_STREAM("[MiniKing]: LeftLimit set to " << config.left_limit);
  }
  if (config.right_limit != config_.right_limit || first_config_) {
    needs_update = true;
    config_.right_limit = config.right_limit;
    mk_->setRightLim(config.right_limit);
    ROS_INFO_STREAM("[MiniKing]: RightLimit set to " << config.right_limit);
  }
  if (config.gain != config_.gain || first_config_) {
    needs_update = true;
    config_.gain = config.gain;
    mk_->setGain(config.gain);
    ROS_INFO_STREAM("[MiniKing]: Gain set to " << config.gain);
  }
  if (config.bins != config_.bins || first_config_) {
    needs_update = true;
    config_.bins = config.bins;
    mk_->setBins(config.bins);
    ROS_INFO_STREAM("[MiniKing]: Bins set to " << config.bins);
  }

  if (needs_update) {
    mk_->updateConfig();
    printConfigurations();
  }
  config = config_;
  if (first_config_) first_config_ = false;
}

int MiniKingRos::getFrequency(const std::string& s) {
  size_t i;
  for (i = 0; i < Kfrequency; i++) {
    if (s.compare(std::string(frequency_char[i])) == 0)
      break;
  }
  return frequency_int[i];
}

const char* MiniKingRos::getFrequencyChar(const int f) {
  size_t i;
  for (i = 0; i < Kfrequency; i++) {
    if (f == frequency_int[i])
      break;
  }
  return frequency_char[i];
}

int MiniKingRos::getType(const std::string& s) {
  size_t i;
  for (i = 0; i < Ksonar_type; i++) {
    if (s.compare(std::string(sonar_type_char[i])) == 0)
      break;
  }
  return sonar_type_int[i];
}

const char* MiniKingRos::getTypeChar(const int t) {
  size_t i;
  for (i = 0; i < Ksonar_type; i++) {
    if (t == sonar_type_int[i])
      break;
  }
  return sonar_type_char[i];
}

int MiniKingRos::getResolution(const std::string& s) {
  size_t i;
  for (i = 0; i < Kresolution; i++) {
    if (s.compare(std::string(resolution_char[i])) == 0)
      break;
  }
  return resolution_int[i];
}

float MiniKingRos::getResolutionValue(const int r) {
  size_t i;
  for (i = 0; i < Kresolution; i++) {
    if (r == resolution_int[i])
      break;
  }
  return resolution_value[i];
}

const char* MiniKingRos::getResolutionChar(const int r) {
  size_t i;
  for (i = 0; i < Kresolution; i++) {
    if (r == resolution_int[i])
      break;
  }
  return resolution_char[i];
}
