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
static int resolution_int[] = {0, 325, 580, 675, 795, 935, 1210, 200, 1700, 2000, 500, 1500, 295};
static int frequency_int[] = {32, 16, 8,  4};

/**
 * @brief MiniKingRos constructor
 *
 * @param nh Public node handle
 * @param nhp Private node handle
 */
MiniKingRos::MiniKingRos(ros::NodeHandle nh, ros::NodeHandle nhp)
    : nh_(nh), nhp_(nhp) {
  // Get the MiniKing port
  nhp_.param("port",          port_,     std::string("/dev/ttyUSB0"));
  // Init sonar
  mk_ = new MiniKing(const_cast<char*>(port_.c_str()), 0);
  mk_->initSonar();

  // Start dynamic_reconfigure & run configure()
  dynamic_reconfigure::Server<DynConfig>::CallbackType f;
  f = boost::bind(&MiniKingRos::updateConfig, this, _1, _2);
  reconfigure_server_.setCallback(f);

  // Get rest of params
  std::string resolution, type, freq;
  nhp_.param("continuous",    config_.continuous,     true);
  nhp_.param("inverted",      config_.inverted,      false);
  nhp_.param("stare",         config_.stare,         false);
  nhp_.param("disable_motor", config_.disable_motor, false);
  nhp_.param("disable_trans", config_.disable_trans, false);
  nhp_.param("range",         config_.range,            30);
  nhp_.param("left_limit",    config_.left_limit,        0);
  nhp_.param("right_limit",   config_.right_limit,       0);
  nhp_.param("gain",          config_.gain,             40);
  nhp_.param("bins",          config_.bins,            300);
  nhp_.param("resolution",    resolution, std::string("Medium"));
  nhp_.param("type",          type,       std::string("Imaging"));
  nhp_.param("frequency",     freq,       std::string("f675"));

  config_.resolution = getResolution(resolution);
  config_.type       = getType(type);
  config_.frequency  = getFrequency(freq);

  // Override default params
  updateConfig(config_, 0);

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
    "\n\t* Resolution:   " << config.resolution <<
    "\n\t* Continuous:   " << config.continuous <<
    "\n\t* Inverted:     " << config.inverted <<
    "\n\t* Stare:        " << config.stare <<
    "\n\t* DisableMotor: " << config.disable_motor <<
    "\n\t* Resolution:   " << config.resolution <<
    "\n\t* SonarType:    " << config.type <<
    "\n\t* Frequency:    " << config.frequency <<
    "\n\t* Range:        " << config.range <<
    "\n\t* LeftLimit:    " << config.left_limit <<
    "\n\t* RightLimit:   " << config.right_limit <<
    "\n\t* Gain:         " << config.gain <<
    "\n\t* Bins:         " << config.bins);
}

void MiniKingRos::timerCallback(const ros::TimerEvent& event) {
  boost::mutex::scoped_lock lock(config_mutex_);
  miniking_ros::AcousticBeam beam;
  beam.header.frame_id = "sonar";
  beam.header.stamp = ros::Time::now();
  beam.range_max = config_.range;
  beam.bins = config_.bins;
  beam.angle = mk_->getPosition();
  unsigned char *data = mk_->getScanLine();
  for (int i = 0; i < mk_->getDataLength(); i++)
    beam.intensities.push_back(data[i]);
  pub_.publish(beam);
}

void MiniKingRos::updateConfig(DynConfig& config, uint32_t level) {
  boost::mutex::scoped_lock lock(config_mutex_);
  bool forced = false;
  if (level == 0)
    forced = true;

  bool needs_update = false;

  if (config.resolution != config_.resolution || forced) {
    needs_update = true;
    config_.resolution = config.resolution;
    mk_->setResolution(static_cast<Resolution>(config.resolution));
    ROS_INFO_STREAM("[MiniKing]: Resolution set to " << config.resolution);
  }
  if (config.continuous != config_.continuous || forced) {
    needs_update = true;
    config_.continuous = config.continuous;
    mk_->setContinuous(config.continuous);
    ROS_INFO_STREAM("[MiniKing]: Continuous set to " << config.continuous);
  }
  if (config.inverted != config_.inverted || forced) {
    needs_update = true;
    config_.inverted = config.inverted;
    mk_->setInverted(config.inverted);
    ROS_INFO_STREAM("[MiniKing]: Inverted set to " << config.inverted);
  }
  if (config.stare != config_.stare || forced) {
    needs_update = true;
    config_.stare = config.stare;
    mk_->setStare(config.stare);
    ROS_INFO_STREAM("[MiniKing]: Stare set to " << config.stare);
  }
  if (config.disable_motor != config_.disable_motor || forced) {
    needs_update = true;
    config_.disable_motor = config.disable_motor;
    mk_->setMotorDisabled(config.disable_motor);
    ROS_INFO_STREAM("[MiniKing]: DisableMotor set to " << config.disable_motor);
  }
  if (config.disable_trans != config_.disable_trans || forced) {
    needs_update = true;
    config_.disable_trans = config.disable_trans;
    mk_->setTransmitDisabled(config.disable_trans);
    ROS_INFO_STREAM("[MiniKing]: Resolution set to " << config.resolution);
  }
  if (config.type != config_.type || forced) {
    needs_update = true;
    config_.type = config.type;
    mk_->setSonarType(static_cast<SonarType>(config.type));
    ROS_INFO_STREAM("[MiniKing]: SonarType set to " << config.type);
  }
  if (config.frequency != config_.frequency || forced) {
    needs_update = true;
    config_.frequency = config.frequency;
    mk_->setFrequency(static_cast<Frequency>(config.frequency));
    ROS_INFO_STREAM("[MiniKing]: Frequency set to " << config.frequency);
  }
  if (config.range != config_.range || forced) {
    needs_update = true;
    config_.range = config.range;
    mk_->setRange(config.range);
    ROS_INFO_STREAM("[MiniKing]: Range set to " << config.range);
  }
  if (config.left_limit != config_.left_limit || forced) {
    needs_update = true;
    config_.left_limit = config.left_limit;
    mk_->setLeftLim(config.left_limit);
    ROS_INFO_STREAM("[MiniKing]: LeftLimit set to " << config.left_limit);
  }
  if (config.right_limit != config_.right_limit || forced) {
    needs_update = true;
    config_.right_limit = config.right_limit;
    mk_->setRightLim(config.right_limit);
    ROS_INFO_STREAM("[MiniKing]: RightLimit set to " << config.right_limit);
  }
  if (config.gain != config_.gain || forced) {
    needs_update = true;
    config_.gain = config.gain;
    mk_->setGain(config.gain);
    ROS_INFO_STREAM("[MiniKing]: Gain set to " << config.gain);
  }
  if (config.bins != config_.bins || forced) {
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
}

int MiniKingRos::getFrequency(const std::string& s) {
  size_t i;
  for (i = 0; i < Kfrequency; i++) {
    if (s.compare(std::string(frequency_char[i])) == 0)
      break;
  }
  return frequency_int[i];
}

int MiniKingRos::getType(const std::string& s) {
  size_t i;
  for (i = 0; i < Ksonar_type; i++) {
    if (s.compare(std::string(sonar_type_char[i])) == 0)
      break;
  }
  return sonar_type_int[i];
}

int MiniKingRos::getResolution(const std::string& s) {
  size_t i;
  for (i = 0; i < Kresolution; i++) {
    if (s.compare(std::string(resolution_char[i])) == 0)
      break;
  }
  return resolution_int[i];
}
