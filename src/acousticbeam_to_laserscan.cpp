#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <miniking_ros/AcousticBeam.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <auv_msgs/NavSts.h>
#include <sensor_msgs/LaserScan.h>
#include <miniking_ros/peak_detector.h>

using namespace std;

class AcousticbeamToLaserScan
{
public:
    AcousticbeamToLaserScan(ros::NodeHandle nh);

protected:
    void sonarCb(const miniking_ros::AcousticBeamConstPtr beam);
private:
    ros::Publisher ls_pub_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    // Operational variables
    bool first_execution_;
    double right_limit_;
    double left_limit_;
    double angle_step_;
    double angle_;
    
    // Node parameters
    string robot_frame_id_;
    string sonar_frame_id_;
    sensor_msgs::LaserScan ls_;

    int cnt_;

    //obstacle noise
    float old_distance_;
    float noise_threshold_;
    
}; 

AcousticbeamToLaserScan::AcousticbeamToLaserScan(ros::NodeHandle nh) : nh_(nh), cnt_(0)
{
  // Get params
  ros::NodeHandle nhp("~");
  nhp.param("robot_frame_id", robot_frame_id_, string(""));
  nhp.param("sonar_frame_id", sonar_frame_id_, string(""));
 
  first_execution_ = true;
  ls_pub_ = nhp.advertise<sensor_msgs::LaserScan>("sonar_scans", 1);

  // Sonar subscription
  sub_ = nh_.subscribe("/sonar", 20, &AcousticbeamToLaserScan::sonarCb, this);
}


void AcousticbeamToLaserScan::sonarCb(const miniking_ros::AcousticBeamConstPtr beam)
{
  //beam angles conversion from degrees to radians
  right_limit_ = beam->right_limit*(M_PI/180);
  left_limit_ = beam->left_limit*(M_PI/180);
  angle_step_ = beam->angle_step*(M_PI/180);
  angle_ = beam->angle*(M_PI/180);

  // Setup laser scan
  ls_.header.stamp = beam->header.stamp;
  ls_.header.frame_id = beam->header.frame_id;
  ls_.angle_min = (right_limit_+ M_PI/2);
  ls_.angle_max= (left_limit_+M_PI/2);
  ls_.angle_increment = angle_step_;
  ls_.time_increment = 1/30; //f=30Hz
  double angle_size = (right_limit_ - left_limit_)/ angle_step_;
  ls_.scan_time = angle_size*(1/30);
  ls_.range_max = 20.0;//meters
  ls_.range_min = 2.0;

  if (first_execution_ || (ls_.ranges.size() != angle_size))
  {
    ls_.ranges.resize(angle_size);
    ls_.intensities.resize(angle_size);
  }
    
  float distance_btw_bins = beam->range_max/beam->bins;
  int intensity_threshold = 8;
  int peak_threshold = 200;

  std::vector<unsigned char> intensities(beam->intensities);
  for (int i=0; i<intensity_threshold; i++)
    intensities[i]=1;

  double min, max;
  cv::Point min_idx, max_idx;
  cv::minMaxLoc(intensities, &min, &max, &min_idx, &max_idx);
  WeightedMeanPeakDetector pd(20);
  double subpix_max = pd.detect(intensities, max_idx.x);
  max_idx.x = (int) subpix_max;
  float distance = distance_btw_bins*(max_idx.x);

  if(max<peak_threshold)
    max = 1;

  // set max_intensities vector to 1
  std::vector<unsigned char> max_intensities;
  for (int i=0; i<(beam->bins); i++)
    max_intensities.push_back(1);

  // set te max value of intensity in the right position 
  max_intensities[max_idx.x] = max;
  double angle_rad = beam->angle*M_PI/180;
  int angle_idx = (angle_ - left_limit_)/angle_step_ + 0.5;
  for (size_t i = 0; i < ls_.ranges.size(); i++)
  {
    ls_.ranges[i] = 0;
    ls_.intensities[i] = 0;
  }
  
  // Remove obstacle noise
  old_distance_;
  noise_threshold_=1.0;
  if(distance<old_distance_+noise_threshold_ && distance>old_distance_-noise_threshold_)
  {
    ls_.ranges[angle_idx] = distance;
    ls_.intensities[angle_idx] = intensities[max_idx.x];
  }
  else
  {
    ls_.ranges[angle_idx] = 0;
    ls_.intensities[angle_idx] = 0;
  }

  old_distance_ = distance;
  ls_pub_.publish(ls_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "acousticbeam_to_laserscan");
    ros::NodeHandle nh;
    AcousticbeamToLaserScan node(nh);
    ros::spin();
    ros::shutdown();
    return 0;
}
