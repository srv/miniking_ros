#include <miniking_ros/AcousticBeam.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <vector>
#include <tf/transform_datatypes.h>

cv::Mat image, image_square,detections;
int bins = 0;
void callback(const miniking_ros::AcousticBeamConstPtr beam) {
//-----------------------------------------------------------------------------------
float distance_btw_bins = beam->range_max/beam->bins;
int intensity_threshold = 8;
int peak_threshold=80;


  std::vector<unsigned char> intensities(beam->intensities);
  for (int i=0; i<intensity_threshold; i++){
    intensities[i]=1;
  }
  double min, max;
  cv::Point min_idx, max_idx;
  cv::minMaxLoc(intensities, &min, &max, &min_idx, &max_idx);
  float distance = distance_btw_bins*(max_idx.x);
  if(max<peak_threshold){
    max=1;
  }
  
   if(distance<3 && max >= peak_threshold){
    std::cout << "EMERGENCY SURFACE"  << std::endl;
   }
  // std::cout << "INTENSITY:" << max << "  POSITION:" << max_idx<< "  DISTANCE:" << distance  << "  ANGLE:"<< beam->angle << std::endl;
  std::vector<unsigned char> max_intensities;
  // set max_intensities vector to 1
  for (int i=0; i<(beam->bins); i++){
    max_intensities.push_back(1);
  }
  // set te max value of intensity in the position 
  max_intensities[max_idx.x]=max;
  for(int i=0;i<max_intensities.size();i++){    
    // std::cout <<"POSITION:"<<i<< "   MAX_INTENSITY: " << max_intensities[i]-1 <<"   ANGLE: " << beam->angle << std::endl;
    }

//-----------------------------------------------------------------------------------
  cv::namedWindow(beam->header.frame_id, 0);
  cv::namedWindow(beam->header.frame_id+"_square", 0);
  
  if (beam->bins != bins) {
    
    image = cv::Mat::zeros(2*beam->bins + 1, 2*beam->bins+1, CV_8UC1);
    image_square = cv::Mat::zeros(beam->bins, 360, CV_8UC1);
    detections = cv::Mat::zeros(2*beam->bins + 1, 2*beam->bins+1, CV_8UC1); 
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
    detections.at<unsigned char>(y, x) = max_intensities[i];

    // image.at<unsigned char>(y, x) = intensities[i];
  }
  // Holds the colormap version of the image:
  cv::Mat cm_image, cm_image_square, cm_detections;
  // Apply the colormap:
  cv::applyColorMap(image, cm_image, cv::COLORMAP_JET);
  cv::applyColorMap(image_square, cm_image_square, cv::COLORMAP_JET);
  cv::applyColorMap(detections, cm_detections, cv::COLORMAP_JET);

  // Show the result:
  cv::imshow(beam->header.frame_id, cm_image);
  cv::imshow(beam->header.frame_id+"_square", cm_image_square);
  cv::imshow(beam->header.frame_id+"_detections", cm_detections);
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
