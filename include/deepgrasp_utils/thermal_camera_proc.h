#ifndef PROJECT_THERMAL_CAMERA_PROC_H
#define PROJECT_THERMAL_CAMERA_PROC_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

class ThermalCameraProc {
 public:
  ThermalCameraProc(ros::NodeHandle, bool invert, bool thresh,
      int cold_temp=-1, float agc_border=0.1, int output_height=-1,
      int output_width=-1, bool constant_norm=false);
  void image_cb(const sensor_msgs::ImageConstPtr &im_in);

 private:
  ros::NodeHandle nh;
  boost::shared_ptr<image_transport::ImageTransport> it;
  image_transport::Publisher pub;
  image_transport::Subscriber sub;
  bool invert, thresh;
  int cold_temp;
  cv::Mat agc_mask;
  float agc_border;
  int output_height, output_width;
  double min_px, max_px;
  bool constant_norm;
};

#endif  // PROJECT_THERMAL_CAMERA_PROC_H
