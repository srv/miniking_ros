/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <miniking_ros/AcousticBeam.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <string>

cv::Mat image, image_square;
int bins = 0;
double range = 10;

void callback(const miniking_ros::AcousticBeamConstPtr beam) {
  cv::namedWindow(beam->header.frame_id, 0);
  cv::namedWindow(beam->header.frame_id+"_square", 0);
  double range_step = beam->range_max/beam->bins;
  int bin_count = range / range_step;
  if (beam->bins != bins) {
    image = cv::Mat::zeros(2*bin_count + 1, 2*bin_count+1, CV_8UC1);
    image_square = cv::Mat::zeros(bin_count, 360, CV_8UC1);
    bins = beam->bins;
  }
  double angle = beam->angle;
  for (size_t i = 0; i < bin_count; i++) {
    double x, y;
    double mag = i;
    x = bin_count + mag*cos(angle*M_PI/180.0);
    y = bin_count + mag*sin(angle*M_PI/180.0);
    image.at<unsigned char>(y, x) = beam->intensities[i];
    image_square.at<unsigned char>(i, static_cast<int>(angle)) = beam->intensities[i];
  }
  // Holds the colormap version of the image:
  cv::Mat cm_image, cm_image_square;
  // Apply the colormap:
  cv::applyColorMap(image, cm_image, cv::COLORMAP_JET);
  cv::applyColorMap(image_square, cm_image_square, cv::COLORMAP_JET);
  // Show the result:
  cv::imshow(beam->header.frame_id, cm_image);
  cv::imshow(beam->header.frame_id+"_square", cm_image_square);
  cv::waitKey(3);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "acoustic_beam_viewer");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  std::string topic;
  nhp.param("input", topic, std::string("/miniking_node/sonar"));
  ros::Subscriber sub = nh.subscribe<miniking_ros::AcousticBeam>(topic, 10, callback);
  ros::spin();
  return 0;
}
