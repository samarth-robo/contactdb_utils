//
// Created by samarth on 10/16/18.
//

#ifndef PROJECT_REGISTER_DEPTH_H
#define PROJECT_REGISTER_DEPTH_H

#include <kinect2_registration/kinect2_registration.h>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace cv;
typedef sensor_msgs::CameraInfo CInfo;
typedef sensor_msgs::Image Im;

class RegisterDepth {
 private:
  boost::shared_ptr<DepthRegistration> depth_reg;
  ros::NodeHandle nh;
  image_transport::ImageTransport *it;

  // subscribers
  image_transport::Subscriber depth_im_sub;
  ros::Subscriber depth_cinfo_sub, thermal_cinfo_sub;

  // publishers
  image_transport::Publisher depth_im_pub;

  // callbacks
  void depth_cinfo_cb(const CInfo::ConstPtr &in);
  void thermal_cinfo_cb(const CInfo::ConstPtr &in);
  void tf2_listener();

  // data
  bool depth_cinfo_received, thermal_cinfo_received, tf_received;
  bool registration_initialized;
  CInfo::Ptr depth_cinfo, thermal_cinfo;
  tf2::Stamped<tf2::Transform> tform;
  cv::Mat K_depth, K_thermal;
  cv::Size size_depth, size_thermal;

  // tf
  tf2_ros::Buffer tf_buffer;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener;

 public:
  RegisterDepth();
  ~RegisterDepth();
  void depth_im_cb(const Im::ConstPtr &depth);
};

#endif //PROJECT_REGISTER_DEPTH_H
