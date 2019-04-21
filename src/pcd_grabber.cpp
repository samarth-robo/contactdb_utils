//
// Created by samarth on 4/9/17.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/highgui.hpp>
#include <pcl/point_types_conversion.h>
#include <ros/callback_queue.h>
#include <cv_bridge/cv_bridge.h>

using namespace pcl;
using namespace std;
using namespace ros;

class PCDGrabber {
  typedef PointXYZI PointT;
  typedef PointCloud<PointT> PointCloudT;
  PointCloudT::Ptr cloud;
  cv_bridge::CvImageConstPtr image;
  boost::mutex pc_mutex, image_mutex;
  size_t count = 0;
  string save_dir, object_name;
  bool cv_window_init;

  static void mouse_cb(int event, int x, int y, int flags, void *param);

 public:
  PCDGrabber(const string &save_dir, const string &object_name);
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
  void image_cb(const sensor_msgs::ImageConstPtr &input);
};

PCDGrabber::PCDGrabber(const string &save_dir, const string &object_name) :
    cloud(new PointCloudT), cv_window_init(false), save_dir(save_dir),
    object_name(object_name) {
  cv::startWindowThread();
  cv::namedWindow("focus");
  cv::setMouseCallback("focus", &PCDGrabber::mouse_cb, this);
}

void PCDGrabber::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
  pc_mutex.lock();
  fromROSMsg(*input, *cloud);
  pc_mutex.unlock();
  cv::waitKey(1);
}

void PCDGrabber::image_cb(const sensor_msgs::ImageConstPtr &input) {
  image_mutex.lock();
  image = cv_bridge::toCvShare(input);
  image_mutex.unlock();
}

void PCDGrabber::mouse_cb(int event, int x, int y, int flags,
                                 void *param) {
  if (event != cv::EVENT_LBUTTONDOWN) return;

  // hack to access non-static class members
  auto *this_ = reinterpret_cast<PCDGrabber *>(param);

  stringstream ss;
  ss << this_->save_dir << "/" << this_->object_name << "/"
     << setw(5) << setfill('0') << this_->count;
  string cloud_filename = ss.str() + string(".pcd");
  string image_filename = ss.str() + string(".png");

  {
    boost::lock_guard<boost::mutex> guard(this_->pc_mutex);
    if (pcl::io::savePCDFileBinary(cloud_filename, *(this_->cloud)) == 0) {
      ROS_INFO("Cloud %s saved!", cloud_filename.c_str());
      this_->count++;
    } else ROS_ERROR("Could not save point cloud!");
  }
  {
    boost::lock_guard<boost::mutex> guard(this_->image_mutex);
    if (cv::imwrite(image_filename, this_->image->image))
      ROS_INFO("Image %s saved!", image_filename.c_str());
    else ROS_ERROR("Could not save image!");
  }
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "pcd_grabber");
  string save_dir, object_name;
  if (argc != 3) {
    ROS_ERROR("Usage: %s save_dir object_name", argv[0]);
    return -1;
  } else {
    save_dir = string(argv[1]);
    object_name = string(argv[2]);
  }
  PCDGrabber pcd_grabber(save_dir, object_name);

  ros::NodeHandle nh;
  /*
  // Using AsyncSpinners, but does not work :/
  // options for the two subscribers
  CallbackQueue q_cloud, q_image;
  SubscribeOptions ops_cloud = SubscribeOptions::create<sensor_msgs::PointCloud2>
      ("points", 5, &PCDGrabber::cloud_cb, &pcd_grabber, &q_cloud);
  SubscribeOptions ops_image = SubscribeOptions::create<sensor_msgs::Image>
      ("image",  5, &PCDGrabber::image_cb, &pcd_grabber, &q_image);
  Subscriber sub_cloud = nh.subscribe(ops_cloud), sub_image = nh.subscribe(ops_image);
  // Async spinners
  AsyncSpinner spinner_cloud(1, &q_cloud), spinner_image(1, &q_image);
  spinner_cloud.start();
  spinner_image.start();
  */
  Subscriber sub_cloud = nh.subscribe("points", 5, &PCDGrabber::cloud_cb,
                                      &pcd_grabber);
  Subscriber sub_image = nh.subscribe("image", 5, &PCDGrabber::image_cb,
                                      &pcd_grabber);


  if (!ros::ok()) {
    ROS_ERROR("Failed to initialize %s", argv[0]);
    return -1;
  } else {
    ROS_INFO("pcd_grabber started, save_dir=%s, object_name=%s.",
        save_dir.c_str(), object_name.c_str());
    ROS_INFO("Press 's' or spacebar to save");
  }

  ros::spin();

  return 0;
}