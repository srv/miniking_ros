#include <miniking_ros/AcousticBeam.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <vector>
#include <tf/transform_datatypes.h>

cv::Mat image, image_square,detections;
int bins = 0;
int count=1;
int peak_threshold = 50;

float old_distance=0;
float noise_threshold = 1.0;
bool first_execution=true;

void callback(const miniking_ros::AcousticBeamConstPtr beam) {
  float distance_btw_bins = beam->range_max/beam->bins;
  float min_radius = 0.8; // meters
  int min_radius_idx = min_radius/distance_btw_bins;


  std::vector<unsigned char> intensities(beam->intensities);

  // Remove first readouts,
  for (int i = 0; i < min_radius_idx; i++) {
    intensities[i] = 0;
  }

  double min, max;
  cv::Point min_pt, max_pt;
  int min_idx, max_idx;
  cv::minMaxLoc(intensities, &min, &max, &min_pt, &max_pt);
  min_idx = min_pt.x;
  max_idx = max_pt.x;
  float distance = distance_btw_bins*(max_idx);
  // std::cout << "INTENSITY:" << max << "  POSITION:" << max_idx<< "  DISTANCE:" << distance  << "  ANGLE:"<< beam->angle << std::endl;
  // std::cout << count <<" distance: "<< distance<< "  old distance: "<< old_distance<< std::endl;
  std::vector<unsigned char> safety_intensities(beam->bins, 0);

  //SAFETY
  double safety_distance = 3; //meters
  int bins_to_draw = safety_distance/distance_btw_bins;

  for (int i = 0; i <= bins_to_draw; i++) {
    if (max_idx - i > 0)
      safety_intensities[max_idx-i] = 255;
  }
  // // ROS_INFO_STREAM("Distance is " << distance << " max is " << max);
  // if (distance < safety_distance && max >= peak_threshold) {
  //   std::cout << "EMERGENCY SURFACE"  << std::endl;
  // }

  cv::namedWindow(beam->header.frame_id, 0);
  cv::namedWindow(beam->header.frame_id+"_square", 0);
  cv::namedWindow(beam->header.frame_id+"_detections", 0);

  if (beam->bins != bins) {
    image = cv::Mat::zeros(2*beam->bins + 1, 2*beam->bins+1, CV_8UC1);
    image_square = cv::Mat::zeros(beam->bins, 360, CV_8UC1);
    detections = cv::Mat::zeros(2*beam->bins + 1, 2*beam->bins+1, CV_8UC3);
    bins = beam->bins;
  }

  for (size_t i = 0; i < beam->bins; i++) {
    double x, y;
    double mag = i;
    float angle= beam->angle;
    x = beam->bins + mag*cos(angle*M_PI/180.0);
    y = beam->bins + mag*sin(angle*M_PI/180.0);
    image.at<unsigned char>(y, x) = beam->intensities[i];
    image_square.at<unsigned char>(i, static_cast<int>(angle)) = beam->intensities[i];

    if (max >= peak_threshold) {
      detections.at<cv::Vec3b>(y, x) = cv::Vec3b(safety_intensities[i], 0, 0); // BGR
    }
  }

  // Draw max point
  if (max >= peak_threshold) {
    double x, y;
    x = beam->bins + max_idx*cos(beam->angle*M_PI/180.0);
    y = beam->bins + max_idx*sin(beam->angle*M_PI/180.0);
    detections.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
  }

  // Draw robot
  cv::circle(detections, cv::Point(beam->bins, beam->bins), 3, cv::Scalar(255, 255, 255), 3);

  // Holds the colormap version of the image:
  cv::Mat cm_image, cm_image_square,cm_detections;
  // Apply the colormap:
  cv::applyColorMap(image, cm_image, cv::COLORMAP_JET);
  cv::applyColorMap(image_square, cm_image_square, cv::COLORMAP_JET);
  cv::applyColorMap(detections, cm_detections, cv::COLORMAP_JET);

  // Show the result:
  cv::imshow(beam->header.frame_id, cm_image);
  cv::imshow(beam->header.frame_id+"_square", cm_image_square);
  cv::imshow(beam->header.frame_id+"_detections", detections);
  cv::namedWindow("detections", 1);
  cvCreateTrackbar("PEAK THRESHOLD","detections",&peak_threshold, 255);
  // createTrackbar( TrackbarName, "PEAK THRESHOLD", peak_threshold&, 200, on_trackbar );
  // sprintf( TrackbarName, "PEAK THRESHOLD %d", peak_threshold);
  // createTrackbar( TrackbarName, "PEAK THRESHOLD", peak_threshold&, 200, on_trackbar);
  // Show some stuff
  // on_trackbar(peak_threshold, 0 );
  cv::waitKey(3);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "acousticbeam_viewer");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  std::string topic;
  nhp.param("input", topic, std::string("/miniking_node/sonar"));
  ros::Subscriber sub = nh.subscribe<miniking_ros::AcousticBeam>(topic, 10, callback);
  ros::spin();
  return 0;
}
