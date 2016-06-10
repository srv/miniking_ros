#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <occupancy_grid_utils/exceptions.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <exception>
#include <algorithm>
#include <iostream>
#include <vector>
#include <iterator>
#include <miniking_ros/ObstacleDistance.h>
#include <miniking_ros/miniking_ros.h>

using namespace occupancy_grid_utils;
using namespace std;

class ObstacleDetector
{

private:

  // Operational variables
  bool tf_catched_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Static transforms
  double sparus_to_sonar_;

  // Node parameters
  double yaw_aperture_;
  double obstacle_prob_th_;
  double yaw_step_;
  string sonar_frame_id_;
  string robot_frame_id_;
  ros::Publisher obstacle_distance_pub_;
public:

  ObstacleDetector(): tf_catched_(false)
  {
    ros::NodeHandle nhp("~");
    // PARAMS:
    nhp.param("yaw_aperture", yaw_aperture_, 2.0);
    nhp.param("yaw_step", yaw_step_, 1.0);
    nhp.param("obstacle_prob_th", obstacle_prob_th_, 20.0);
    nhp.param("sonar_frame_id", sonar_frame_id_, string("sonar"));

    // Convert yaw to rad
    yaw_aperture_ = yaw_aperture_ * (M_PI/180.0);
    yaw_step_ = yaw_step_ * (M_PI/180.0);

    //Publisher
     obstacle_distance_pub_= nhp.advertise<miniking_ros::ObstacleDistance>("obstacle_info", 100);
  }

  void syncCallback (const nav_msgs::OdometryConstPtr& odometry,
                     const nav_msgs::OccupancyGridConstPtr& local_costmap)

  {
    // TF Sparus to sonar
    if (!tf_catched_)
    {
      try
      {
        tf::StampedTransform sonar_tf;
        tf_listener_.lookupTransform(odometry->child_frame_id,
            sonar_frame_id_,
            ros::Time(0),
            sonar_tf);
        sparus_to_sonar_ = sonar_tf.getOrigin().x();
        tf_catched_ = true;
      }
      catch (tf::TransformException ex)
      {
        return;
      }
    }

    // Robot Velocity
    double robot_velocity = odometry->twist.twist.linear.x;

    // Ignore negative velocity
    if(robot_velocity < 0)
      return;

    // Extract occupancy grid info
    nav_msgs::MapMetaData map_data = local_costmap->info;

    // Robot Point
    geometry_msgs::Point robot_point;
    robot_point.x = odometry->pose.pose.position.x;
    robot_point.y = odometry->pose.pose.position.y;
    robot_point.z = odometry->pose.pose.position.z;
    tf::Quaternion q(odometry->pose.pose.orientation.x,
                     odometry->pose.pose.orientation.y,
                     odometry->pose.pose.orientation.z,
                     odometry->pose.pose.orientation.w);
    // Get yaw
    tf::Matrix3x3 rot(q);
    double roll, pitch, yaw;
    rot.getRPY(roll, pitch, yaw);
    while (yaw > 2*M_PI)
      yaw = yaw - 2*M_PI;
    while (yaw < 0)
      yaw = yaw + 2*M_PI;

    // Compute ray
    double resolution = map_data.resolution;
    std::vector<std::pair<size_t, int> > ray;
    for (double angle=yaw-yaw_aperture_; angle<=yaw+yaw_aperture_; angle=angle+yaw_step_)
    {
      // Increment computation
      double m = tan(angle);
      double a = 1.0 + m*m;
      double b = -2.0 * robot_point.x * (1.0 + m*m);
      double c = (robot_point.x * robot_point.x) * (1.0 + m*m) - 2.0*resolution*resolution;
      double sol1 = (-b + sqrt(b*b - 4.0*a*c) ) / (2.0*a);
      double inc = fabs(fabs(sol1) - fabs(robot_point.x));
      if (angle > M_PI/2.0 && angle < 3.0*M_PI/2.0)
        inc = -inc;

      // Initial x
      double x = robot_point.x;
      try
      {
        while (1)
        {
          x = x+inc;
          geometry_msgs::Point ray_point;
          ray_point.y = m*(x-robot_point.x) + robot_point.y;
          ray_point.x = x;
          index_t idx = pointIndex(map_data, ray_point);
          ray.push_back(std::make_pair(idx,local_costmap->data[idx]));
        }

      }
      catch(const CellOutOfBoundsException& e)
      {
        ROS_DEBUG_STREAM("CellOutOfBounds: " << ray.size());
      }
      catch(const std::exception& e)
      {
        ROS_ERROR_STREAM("Unexpected error: " << e.what());
      }
    }

    // Search an obstacle into the ray
    Cell robot_cell = pointCell(map_data, robot_point);
    std::vector<double> obstacle_distances;
    for (uint i=0; i<ray.size(); i++)
    {
      if (ray[i].second > obstacle_prob_th_)
      {
        Cell obstacle_cell = indexCell(map_data, ray[i].first);

        geometry_msgs::Point obstacle_point = cellCenter(map_data, obstacle_cell);

        double resolution = map_data.resolution;
        double x_robot = robot_cell.x*resolution + resolution/2;
        double y_robot = robot_cell.y*resolution + resolution/2;
        double x_obstacle = obstacle_cell.x*resolution + resolution/2;
        double y_obstacle = obstacle_cell.y*resolution + resolution/2;
        double obstacle_distance = sqrt(((x_robot-x_obstacle)*(x_robot-x_obstacle))+((y_robot-y_obstacle)*(y_robot-y_obstacle)));
        obstacle_distances.push_back(obstacle_distance);
      }
    }

    //Compute de minimum distance to obstacle
    if(obstacle_distances.size()== 0)
    {
      obstacle_distances.push_back(std::numeric_limits<double>::max());
    }
    else
    {
      double min_distance = *std::min_element(obstacle_distances.begin(), obstacle_distances.end());
      min_distance = min_distance-sparus_to_sonar_;
      double collision_time = min_distance/robot_velocity;
      miniking_ros::ObstacleDistance obstacle_info;

      obstacle_info.header.frame_id = sonar_frame_id_;
      obstacle_info.header.stamp = ros::Time::now();
      obstacle_info.distance = min_distance;
      obstacle_info.forward_velocity = robot_velocity;
      obstacle_info.collision_time = collision_time;

      obstacle_distance_pub_.publish(obstacle_info);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_detector");
  ros::NodeHandle nh;
  ObstacleDetector obstacle_detector;

  // Odom and local_costmap time sync
  message_filters::Subscriber<nav_msgs::Odometry> odometry_sub(nh,"odometry", 10);
  message_filters::Subscriber<nav_msgs::OccupancyGrid> local_costmap_sub(nh, "local_costmap", 5);

  typedef message_filters::sync_policies::ApproximateTime <nav_msgs::Odometry, nav_msgs::OccupancyGrid> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(5), odometry_sub, local_costmap_sub);
  sync.registerCallback(boost::bind(&ObstacleDetector::syncCallback, &obstacle_detector, _1, _2));

  ros::spin();
  return 0;
}
