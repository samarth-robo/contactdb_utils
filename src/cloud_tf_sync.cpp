#include <boost/filesystem/fstream.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/callback_queue.h>
#include <fstream>

namespace bfs = boost::filesystem;
using namespace pcl;
using namespace ros;
using namespace std;

typedef PointCloud<PointXYZRGB> PC;
typedef sensor_msgs::Image Im;
typedef sensor_msgs::CameraInfo CInfo;

class CloudTFSynchronizer {
  tf2_ros::Buffer tf_buffer;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener;
  image_transport::ImageTransport *it;

  // subscribers
  image_transport::Subscriber rgb_sub, thermal_sub, depth_sub;
  ros::Subscriber cloud_sub, thermal_cinfo_sub;

  ros::NodeHandle nh;
  ros::Time last_time;
  int count;
  string data_dir, target_frame, object_name;

  // callbacks
  void rgb_image_cb(const Im::ConstPtr &in);
  void thermal_image_cb(const Im::ConstPtr &in);
  void depth_image_cb(const Im::ConstPtr &in);
  void thermal_cinfo_cb(const CInfo::ConstPtr &in);
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &in);

  // mutexes
  // boost::mutex rgb_mutex, thermal_mutex, cloud_mutex, depth_mutex;

  // data
  PC::Ptr cloud;
  cv::Mat rgb_image, thermal_image, depth_image;
  CInfo::Ptr thermal_cinfo;
  bool cloud_received, rgb_received, thermal_received, depth_received, cinfo_received;
  bool re_extract;

 public:
  CloudTFSynchronizer(const string &data_dir, const string &object_name,
      const string &target_frame, bool re_extract=false);
  ~CloudTFSynchronizer();
  void data_cb();
  void debug_cb();
};

CloudTFSynchronizer::CloudTFSynchronizer(const string &data_dir,
    const string &object_name, const string &target_frame, bool re_extract):
    nh(ros::NodeHandle()), count(0), data_dir(data_dir),
    object_name(object_name), target_frame(target_frame), cloud(new PC()),
    thermal_cinfo(new CInfo()), re_extract(re_extract),
    cloud_received(false), rgb_received(false), thermal_received(false),
    depth_received(false), cinfo_received(false) {
    tf_listener = boost::make_shared<tf2_ros::TransformListener>(tf_buffer);

    // subscribers
    cloud_sub = nh.subscribe("points", 1, &CloudTFSynchronizer::cloud_cb, this);
    it = new image_transport::ImageTransport(nh);
    rgb_sub = it->subscribe("rgb_image", 1, &CloudTFSynchronizer::rgb_image_cb,
        this);
    thermal_sub = it->subscribe("thermal_image", 1,
        &CloudTFSynchronizer::thermal_image_cb, this);
    depth_sub = it->subscribe("depth_image", 1,
        &CloudTFSynchronizer::depth_image_cb, this);
    thermal_cinfo_sub = nh.subscribe("thermal_camera_info", 1,
        &CloudTFSynchronizer::thermal_cinfo_cb, this);
}

CloudTFSynchronizer::~CloudTFSynchronizer() {
    delete it;
}

void CloudTFSynchronizer::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &in) {
    fromROSMsg(*in, *cloud);
    cloud_received = true;
}

void CloudTFSynchronizer::rgb_image_cb(const Im::ConstPtr &in) {
    rgb_image = cv_bridge::toCvCopy(in, "bgr8")->image;
    rgb_received = true;
}

void CloudTFSynchronizer::thermal_image_cb(const Im::ConstPtr &in) {
    thermal_image = cv_bridge::toCvCopy(in, "bgr8")->image;
    thermal_received = true;
}

void CloudTFSynchronizer::depth_image_cb(const Im::ConstPtr &in) {
    depth_image = cv_bridge::toCvCopy(in)->image;
    depth_received = true;
}

void CloudTFSynchronizer::thermal_cinfo_cb(const CInfo::ConstPtr &in) {
    *thermal_cinfo = *in;
    cinfo_received = true;
}

void CloudTFSynchronizer::data_cb() {
    tf2::Stamped<tf2::Transform> tform;
    try {
        tf2::fromMsg(
            tf_buffer.lookupTransform("turntable_base", target_frame, Time(0)),
            tform);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    if (tform.stamp_ == last_time) return;

    if (!cloud_received) {
        ROS_WARN("PointCloud not received");
        return;
    }
    if (!thermal_received) {
        ROS_WARN("Thermal image not received");
        return;
    }
    if (!rgb_received) {
        ROS_WARN("RGB image not received");
        return;
    }
    if (!depth_received) {
        ROS_WARN("Depth image not received");
        return;
    }
    if (!cinfo_received) {
        ROS_WARN("CameraInfo not received");
        return;
    }
    last_time = tform.stamp_;

    // ROS_DEBUG_STREAM("Cloud TS = " << cloud_time
    //                                << " RGB TS = " << rgb_time
    //                                << " Thermal TS = "<< thermal_time);

    if (count == 0 && !re_extract) {
        // save thermal camera info
        stringstream filename;
        filename << data_dir << "/thermal_images/camera_info.txt";
        ofstream f(filename.str());
        if (!f.is_open()) {
            ROS_ERROR("Could not open %s for writing\n", filename.str().c_str());
            return;
        }
        f << thermal_cinfo->K[0] << " " << thermal_cinfo->K[4] << " "
          << thermal_cinfo->K[2] << " " << thermal_cinfo->K[5] << " "
          << thermal_cinfo->width << " " << thermal_cinfo->height << endl;
        f.close();

        // save turntable base pose
        while (!tf_buffer.canTransform(cloud->header.frame_id, "turntable_base",
            Time(0), Duration(0.1)));
        tf2::Stamped<tf2::Transform> tform_tf;
        tf2::fromMsg(
            tf_buffer.lookupTransform(cloud->header.frame_id, "turntable_base",
            Time(0)), tform_tf);
        const auto &o = tform_tf.getOrigin();
        tf2::Matrix3x3 m(tform_tf.getRotation());

        filename.str("");
        filename << data_dir << "/poses/tt_base.txt";
        f.open(filename.str());
        if (f.is_open()) {
            f << o.x() << " " << o.y() << " " << o.z() << " ";
            f << m[0].x() << " " << m[0].y() << " " << m[0].z() << " ";
            f << m[1].x() << " " << m[1].y() << " " << m[1].z() << " ";
            f << m[2].x() << " " << m[2].y() << " " << m[2].z() << endl;
        }
        else {
            ROS_ERROR("Could not open %s for writing", filename.str().c_str());
            return;
        }
        f.close();
    }

    // save pointcloud and images
    stringstream filename;
    filename << data_dir << "/pointclouds/" << setfill('0') << setw(2) << count
             << ".pcd";
    bool done = io::savePCDFileBinary(filename.str(), *cloud) == 0;

    filename.str("");
    filename << data_dir << "/rgb_images/" << setfill('0') << setw(2) << count
             << ".png";
    done &= cv::imwrite(filename.str(), rgb_image);

    filename.str("");
    filename << data_dir << "/depth_images/" << setfill('0') << setw(2) << count
             << ".png";
    done &= cv::imwrite(filename.str(), depth_image);

    filename.str("");
    filename << data_dir << "/thermal_images/" << setfill('0') << setw(2)
             << count << ".png";
    // cv::Mat im_cmapped;
    // cv::applyColorMap(thermal_image, im_cmapped, cv::COLORMAP_JET);
    // done &= cv::imwrite(filename.str(), im_cmapped);
    done &= cv::imwrite(filename.str(), thermal_image);

    // save pose
    if (!re_extract) {
      filename.str("");
      filename << data_dir << "/poses/tt_frame_" << setfill('0') << setw(2)
        << count << ".txt";
      const auto &o = tform.getOrigin();
      tf2::Matrix3x3 m(tform.getRotation());
      ofstream f(filename.str());
      if (f.is_open()) {
        done &= true;
        f << o.x() << " " << o.y() << " " << o.z() << " ";
        f << m[0].x() << " " << m[0].y() << " " << m[0].z() << " ";
        f << m[1].x() << " " << m[1].y() << " " << m[1].z() << " ";
        f << m[2].x() << " " << m[2].y() << " " << m[2].z() << endl;
      } else done &= false;
      f.close();
    }

    if (done) ROS_INFO("Wrote files for %s pose %d", object_name.c_str(), count+1);
    count++;
}

void CloudTFSynchronizer::debug_cb() {
    tf2::Stamped<tf2::Transform> tform;
    try {
        tf2::fromMsg(
            tf_buffer.lookupTransform("turntable_base", target_frame, Time(0)),
            tform);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    /*
    double roll, pitch, yaw;
    tform.getBasis().getRPY(roll, pitch, yaw);
    yaw *= 180.0 / M_PI;
    */
    ROS_INFO_STREAM("Current time: " << tform.stamp_);
    ROS_INFO_STREAM("Last time: " << last_time);
    if (tform.stamp_ == last_time) return;
    last_time = tform.stamp_;

    count++;

    ROS_INFO("## Callback %d", count);
}

bool create_dir(const string &p) {
    if (bfs::is_directory(p)) {
        cout << "Directory " << p << " already exists" << endl;
        return true;
    } else {
        try {
            return bfs::create_directory(p);
        } catch (bfs::filesystem_error &e) {
            cerr << e.what() << endl;
            return false;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_tf_sync");

    if (argc < 3 || argc > 4) {
        cout << "Usage: ./" << argv[0] << " data_dir object_name [re_extract]" << endl;
        return -1;
    }
    string root_dir = string(argv[1]) + string("/") + string(argv[2]);
    bool re_extract = (argc==4) ? string("true").compare(argv[3])==0 : false;
    if (re_extract) ROS_INFO("Only extracting images and pointclouds");

    // create directories
    bool done = create_dir(root_dir + string("/rgb_images"));
    done &= create_dir(root_dir + string("/thermal_images"));
    done &= create_dir(root_dir + string("/poses"));
    done &= create_dir(root_dir + string("/pointclouds"));
    done &= create_dir(root_dir + string("/depth_images"));
    if (done) ROS_INFO("Created directories");
    else {
        ROS_ERROR("Could not create directories");
        return -1;
    }


    CloudTFSynchronizer cloud_tf_sync(root_dir, argv[2], "turntable_frame",
        re_extract);
    ros::Rate r(60);

    // ros::AsyncSpinner spinner(8);
    // spinner.start();
    while (ros::ok()) {
        cloud_tf_sync.data_cb();
        // cloud_tf_sync.debug_cb();
        r.sleep();
        ros::spinOnce();
    }

    if (re_extract) return 0;

    // create some txt files
    // txt file containing object name
    string object_name(argv[2]);
    object_name = object_name.substr(0, object_name.find_first_of('-'));
    string filename = root_dir + "/object_name.txt";
    ofstream f(filename);
    if (f.is_open()) {
        f << object_name << endl;
        f.close();
    } else ROS_ERROR_STREAM("Could not open " << filename << " for writing");

    // txt file containing excluded views
    filename = root_dir + "/excluded_views.txt";
    f.open(filename);
    if (!f.is_open()) ROS_ERROR_STREAM("Could not create " << filename);
    f.close();

    // object flip information
    filename = root_dir + "/object_flip.txt";
    f.open(filename);
    if (f.is_open()) {
        for (int i = 0; i < 6; i++) f << 0 << " ";
        f << endl;
        f.close();
    } else ROS_ERROR_STREAM("Could not open " << filename << " for writing");

    // first view
    filename = root_dir + "/first_view.txt";
    f.open(filename);
    if (!f.is_open()) ROS_ERROR_STREAM("Could not create " << filename);
    f.close();

    return 0;
}
