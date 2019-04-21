//
// Created by samarth on 10/16/18.
//

#include "register_depth.h"
#include <cv_bridge/cv_bridge.h>

RegisterDepth::RegisterDepth() :
    nh(ros::NodeHandle()),
    depth_cinfo(new CInfo()), thermal_cinfo(new CInfo()),
    depth_cinfo_received(false), thermal_cinfo_received(false),
    tf_received(false), registration_initialized(false),
    depth_reg(DepthRegistration::New(DepthRegistration::Method::OPENCL)) {

  it = new image_transport::ImageTransport(nh);
  depth_cinfo_sub = nh.subscribe("depth_camera_info", 1,
      &RegisterDepth::depth_cinfo_cb, this);
  thermal_cinfo_sub = nh.subscribe("thermal_camera_info", 1,
      &RegisterDepth::thermal_cinfo_cb, this);
  depth_im_sub = it->subscribe("depth_unregistered", 1,
      &RegisterDepth::depth_im_cb, this);
  depth_im_pub = it->advertise("depth_registered", 1);

  tf_listener = boost::make_shared<tf2_ros::TransformListener>(tf_buffer);
  ROS_INFO("Registration constructor done");
}

RegisterDepth::~RegisterDepth() {
  delete it;
}

void RegisterDepth::depth_cinfo_cb(const CInfo::ConstPtr &in) {
  if (!depth_cinfo_received) {
    *depth_cinfo = *in;
    depth_cinfo_received = true;
    ROS_INFO("Depth cinfo received");
  }
}

void RegisterDepth::thermal_cinfo_cb(const CInfo::ConstPtr &in) {
  if (!thermal_cinfo_received) {
    *thermal_cinfo = *in;
    thermal_cinfo_received = true;
    ROS_INFO("Thermal cinfo received");
  }
}

void RegisterDepth::depth_im_cb(const Im::ConstPtr &depth) {
  if (!depth_cinfo_received || !thermal_cinfo_received) return;

  if (!tf_received) {
    try {
      tf2::fromMsg(tf_buffer.lookupTransform(thermal_cinfo->header.frame_id,
          depth_cinfo->header.frame_id, ros::Time(0)),
          tform);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
    tf_received = true;
    ROS_INFO("TF information received");
  }

  // init the registration engine
  if (!registration_initialized) {
    cv::Mat K_thermal(cv::Mat::eye(3, 3, CV_64F)),
        K_depth(cv::Mat::eye(3, 3, CV_64F)), R(3, 3, CV_64F),
        t(3, 1, CV_64F);
    K_thermal.at<double>(0, 0) = thermal_cinfo->K.elems[0];
    K_thermal.at<double>(0, 2) = thermal_cinfo->K.elems[2];
    K_thermal.at<double>(1, 1) = thermal_cinfo->K.elems[4];
    K_thermal.at<double>(1, 2) = thermal_cinfo->K.elems[5];
    K_depth.at<double>(0, 0) = depth_cinfo->K.elems[0];
    K_depth.at<double>(0, 2) = depth_cinfo->K.elems[2];
    K_depth.at<double>(1, 1) = depth_cinfo->K.elems[4];
    K_depth.at<double>(1, 2) = depth_cinfo->K.elems[5];
    R.at<double>(0, 0) = tform.getBasis().getRow(0).x();
    R.at<double>(0, 1) = tform.getBasis().getRow(0).y();
    R.at<double>(0, 2) = tform.getBasis().getRow(0).z();
    R.at<double>(1, 0) = tform.getBasis().getRow(1).x();
    R.at<double>(1, 1) = tform.getBasis().getRow(1).y();
    R.at<double>(1, 2) = tform.getBasis().getRow(1).z();
    R.at<double>(2, 0) = tform.getBasis().getRow(2).x();
    R.at<double>(2, 1) = tform.getBasis().getRow(2).y();
    R.at<double>(2, 2) = tform.getBasis().getRow(2).z();
    t.at<double>(0, 0) = tform.getOrigin().x();
    t.at<double>(1, 0) = tform.getOrigin().y();
    t.at<double>(2, 0) = tform.getOrigin().z();
    registration_initialized =
        depth_reg->init(K_thermal, cv::Size(thermal_cinfo->width, thermal_cinfo->height),
        K_depth, cv::Size(depth_cinfo->width, depth_cinfo->height),
        cv::Mat::zeros(1, 4, CV_64F), R, t);
    if (registration_initialized)
      ROS_INFO("Registration engine ignited!");
    else {
      ROS_ERROR("Could not initialize the registration engine");
      return;
    }
  }

  cv::Mat registered;
  depth_reg->registerDepth(cv_bridge::toCvShare(depth)->image, registered);
  // ROS_INFO("Computed registered image of size %d x %d", registered.cols, registered.rows);
  Im::Ptr msg = cv_bridge::CvImage(depth->header, depth->encoding,
      registered).toImageMsg();
  depth_im_pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "register_depthmaps");
  RegisterDepth rd;
  ros::spin();
  return 0;
}
