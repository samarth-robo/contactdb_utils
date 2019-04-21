//
// Created by samarth on 3/4/17.
//

#include "deepgrasp_utils/thermal_camera_proc.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "thermal_camera");
  ros::NodeHandle nh;
  if (!ros::ok()) {
    ROS_ERROR("Failed to initialize %s", argv[0]);
    return -1;
  }
  bool thermal_invert(false), thermal_thresh(false), const_norm(false);
  int cold_temp(-1), output_h(-1), output_w(-1);
  float agc_border = 0.1f;
  for (int i = 0; i < argc; i++) {
    std::string arg(argv[i]);
    if (arg == "invert")
      thermal_invert = std::string("true") == std::string(argv[i+1]);
    else if (arg == "thresh")
      thermal_thresh = std::string("true") == std::string(argv[i+1]);
    else if (arg == "cold_temp") cold_temp = std::stoi(argv[i+1]);
    else if (arg == "agc_border") agc_border = std::stof(argv[i+1]);
    else if (arg == "h") output_h = std::stoi(argv[i+1]);
    else if (arg == "w") output_w = std::stoi(argv[i+1]);
    else if (arg == "const_norm")
      const_norm = std::string("true") == std::string(argv[i+1]);
  }
  ThermalCameraProc th_driver(nh, thermal_invert, thermal_thresh, cold_temp,
      agc_border, output_h, output_w, const_norm);
  ros::spin();
  return 0;
}
