#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

class laserscan_to_pointcloud{

public:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  ros::Subscriber laser_sub_;
  ros::Publisher scan_pub_;

  laserscan_to_pointcloud(ros::NodeHandle nh, ros::NodeHandle nhp) : 
    nh_(nh), nhp_(nhp)
  {
    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, &laserscan_to_pointcloud::scanCallback, this);
    scan_pub_ = nhp_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*scan_in, cloud);
    //Publish the cloud.
    scan_pub_.publish(cloud);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserscan_to_pointcloud");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  laserscan_to_pointcloud lstopc(nh, nhp);
  ros::spin();
  return 0;
}