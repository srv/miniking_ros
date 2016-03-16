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

// #include "utils/rosutils/DiagnosticHelper.h"
// #include "navigation/ned.h"

using namespace std;

class ObstacleDetector
{
public:
    ObstacleDetector(ros::NodeHandle nh);

protected:
    void sonarCb(const nav_msgs::OdometryConstPtr odom,
                 const miniking_ros::AcousticBeamConstPtr beam);
private:
    ros::Subscriber sonar_sub_;
    ros::Publisher marker_pub_;

    // Operational variables
    bool tf_catched_;
    tf::TransformListener tf_listener_;

    // Node parameters
    string robot_frame_id_;
    string sonar_frame_id_;

    // Static transforms
    tf::StampedTransform sonar_tf_;
};

ObstacleDetector::ObstacleDetector(ros::NodeHandle nh) : tf_catched_(false)
{
    // Get params
    ros::NodeHandle nhp("~");
    nhp.param("robot_frame_id", robot_frame_id_, string(""));
    nhp.param("sonar_frame_id", sonar_frame_id_, string(""));

    // Publish markers
    marker_pub_ = nhp.advertise<visualization_msgs::Marker>("obstacle_points", 1);

     // Odom and sonar time sync
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odometry", 20);
    message_filters::Subscriber<miniking_ros::AcousticBeam> sonar_sub(nh, "sonar", 20);
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, miniking_ros::AcousticBeam> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(20), odom_sub, sonar_sub);
    sync.registerCallback(boost::bind(&ObstacleDetector::sonarCb,this,_1,_2));
}

void ObstacleDetector::sonarCb(const nav_msgs::OdometryConstPtr odom,
                               const miniking_ros::AcousticBeamConstPtr beam)
{

  //STATIC TF ROBOT TO SONAR sonar_tf_
  if (!tf_catched_)
  {
    // Catch all static tf
    try
    {
      tf_listener_.lookupTransform(robot_frame_id_,
          sonar_frame_id_,
          ros::Time(0),
          sonar_tf_);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }

    tf_catched_ = true;
  }

  // Get odom tf
  double tx = odom->pose.pose.position.x;
  double ty = odom->pose.pose.position.y;
  double tz = odom->pose.pose.position.z;
  double qx = odom->pose.pose.orientation.x;
  double qy = odom->pose.pose.orientation.y;
  double qz = odom->pose.pose.orientation.z;
  double qw = odom->pose.pose.orientation.w;
  tf::Vector3 tf_trans(tx, ty, tz);
  tf::Quaternion tf_q (qx, qy, qz, qw);
  tf::Transform vehicle_pose(tf_q, tf_trans);
    
  float distance_btw_bins = beam->range_max/beam->bins;
  int intensity_threshold = 8;
  int peak_threshold=80;

  std::vector<unsigned char> intensities(beam->intensities);
  for (int i=0; i<intensity_threshold; i++)
  {
    intensities[i]=1;
  }

  double min, max;
  cv::Point min_idx, max_idx;
  cv::minMaxLoc(intensities, &min, &max, &min_idx, &max_idx);
  float distance = distance_btw_bins*(max_idx.x);

  if(max<peak_threshold)
  {
    max=1;
  }

  if(distance<3 && max >= peak_threshold)
  {
    std::cout << "DISTANCE: "<< distance<< " EMERGENCY SURFACE"  << std::endl;
  }

  std::vector<unsigned char> max_intensities;
  // set max_intensities vector to 1
  for (int i=0; i<(beam->bins); i++)
  {
    max_intensities.push_back(1);
  }
  // set te max value of intensity in the position 
  max_intensities[max_idx.x]=max;

  //TF POINT OBSTACLE tf::Vector3 point_obstacle(X_tf, Y_tf, 0.0);
  float PI=3.14;
  float angle= beam->angle*PI/180;
  float right_limit=beam->right_limit*PI/180;
  float left_limit=beam->left_limit*PI/180;
  float X_tf;
  float Y_tf;

  if(angle<=PI)
  {
    angle =PI-angle ;
    X_tf = distance*cos(angle);
    Y_tf = distance*sin(angle);
  }

  if(angle>PI)
  {
    angle= angle - PI;
    X_tf = distance*cos(angle);
    Y_tf = -distance*sin(angle);
  }

  // Transform point to world coordinates
  tf::Vector3 point_obstacle_v(X_tf, Y_tf, 0.0);
  tf::Vector3 point_obstacle_w = vehicle_pose * sonar_tf_ * point_obstacle_v;

  // Create the marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = sonar_frame_id_;
  marker.header.stamp = beam->header.stamp;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(20.0);

  geometry_msgs::Pose p;
  p.position.x = point_obstacle_w.getX();
  p.position.y = point_obstacle_w.getY();
  p.position.z = point_obstacle_w.getZ();
  marker.pose = p;
  marker_pub_.publish(marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detector");
    ros::NodeHandle nh;
    ObstacleDetector node(nh);

    ros::spin();
    ros::shutdown();
    return 0;
}
