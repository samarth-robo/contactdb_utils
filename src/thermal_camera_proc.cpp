#include <deepgrasp_utils/thermal_camera_proc.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// clusters gray values of pixels in image
vector<float> cluster_image_temps(cv::Mat const &im, cv::Mat &labels,
    int n_clusters=5) {
  using namespace cv;
  Mat data = im.reshape(1, 1);
  std::vector<float> centers;
  TermCriteria criteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 1.0);
  kmeans(data, n_clusters, labels, criteria, 5, KMEANS_PP_CENTERS, centers);
  return centers;
  /*
  stringstream ss;
  for (auto i : centers) ss << i << " ";
  cout << "Cluster centers: " << ss.str() << endl;
  */
}

ThermalCameraProc::ThermalCameraProc(ros::NodeHandle nh, bool invert,
    bool thresh, int cold_temp, float agc_border, int output_height,
    int output_width, bool constant_norm) :
    nh(nh), it(new image_transport::ImageTransport(nh)),
    invert(invert), thresh(thresh), cold_temp(cold_temp), agc_border(agc_border),
    output_height(output_height), output_width(output_width),
    constant_norm(constant_norm) {
  pub = it->advertise("image_out", 1);
  sub = it->subscribe("image_in", 1, &ThermalCameraProc::image_cb, this);


  ROS_INFO_STREAM("Opened Thermal camera with params:" << endl
      << "cold_temp = " << cold_temp << endl
      << "thresh = " << thresh << endl
      << "invert = " << invert << endl
      << "AGC border = " << agc_border << endl
      << "Constant normalization = " << constant_norm << endl
      << "Output size: " << output_width << " x " << output_height);
}

void ThermalCameraProc::image_cb(const sensor_msgs::ImageConstPtr &im_in) {
  cv_bridge::CvImagePtr im_ptr_out(cv_bridge::toCvCopy(im_in));
  im_ptr_out->encoding = "mono8";
  cv::Mat im(im_ptr_out->image);

  // create AGC mask
  if (agc_mask.empty()) {
    float h = im.rows;
    float w = im.cols;
    agc_mask = Mat(int(h), int(w), CV_8UC1);
    Rect roi(int(agc_border * w),
        int(agc_border * h),
        int((1 - 2 * agc_border) * w),
        int((1 - 2 * agc_border) * h));
    agc_mask(roi) = 255;
    if (constant_norm) {
      cv::minMaxLoc(im, &min_px, &max_px, NULL, NULL, agc_mask);
    }
  }

  // find scaling values
  if (!constant_norm) {
    cv::minMaxLoc(im, &min_px, &max_px, NULL, NULL, agc_mask);
  }
  // min = 0; max = UINT16_MAX;
  double scale = UINT8_MAX / (max_px - min_px), bias = -min_px * scale;

  // rescale the thermal image to make it visible
  if (invert) im.convertTo(im, CV_32F, -scale, UINT8_MAX-bias);
  else im.convertTo(im, CV_32F, scale, bias);

  if (thresh || (cold_temp > 0)) {
    cv::Mat cluster_labels;
    vector<float> cluster_cs = cluster_image_temps(im, cluster_labels);
    long cold_idx;
    if (invert) { // cold -> bright
      cold_idx = max_element(cluster_cs.cbegin(), cluster_cs.cend()) -
          cluster_cs.begin();
    } else {  // cold -> dark
      cold_idx = min_element(cluster_cs.cbegin(), cluster_cs.cend()) -
          cluster_cs.begin();
    }

    if (cold_temp > 0) {  // regulate coldest object temperature
      scale = cold_temp / cluster_cs[cold_idx];
      cv::Mat im_reg;
      im.convertTo(im_reg, CV_8U, scale);
      cluster_cs[cold_idx] *= scale;
      im_reg.copyTo(im, cluster_labels==cold_idx);
    } else im.convertTo(im, CV_8U);

    if (thresh) {  // threshold the image
      if (invert) {
        float t = 0.9f * cluster_cs[cold_idx];
        im = im < t;
      } else {
        float t = 1.1f * cluster_cs[cold_idx];
        im = im > t;
      }
    }
  } else im.convertTo(im, CV_8U);

  if (output_height > 0 &&
      output_width > 0  &&
      (im.rows != output_height || im.cols != output_width)) {
    ROS_INFO_THROTTLE(10, "Resizing thermal images");
    if (thresh) {
      cv::resize(im, im_ptr_out->image,
          cv::Size(output_width, output_height),
          0, 0, cv::INTER_NEAREST);  // nearest is good for binary resizing
    } else {
      cv::resize(im, im_ptr_out->image,
          cv::Size(output_width, output_height),
          0, 0, cv::INTER_CUBIC);  // cubic is good for up-scaling
    }
  } else im_ptr_out->image = im;

  pub.publish(im_ptr_out->toImageMsg());
}
