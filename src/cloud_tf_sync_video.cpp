#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>

namespace bfs = boost::filesystem;
using namespace ros;
using namespace std;
using namespace message_filters;
using namespace image_transport;

typedef sensor_msgs::Image Im;
typedef sensor_msgs::ImageConstPtr ImConstPtr;

class CloudTFVideoSynchronizer {
  tf2_ros::Buffer tf_buffer;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener;

  // subscribers
  SubscriberFilter rgb_sub, thermal_sub, depth_sub;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  int count, count_skip;
  string target_frame;
  bfs::path output_dir;
  bool start;

  // sync policy
  typedef sync_policies::ApproximateTime<Im, Im, Im> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync;

 public:
  CloudTFVideoSynchronizer(const bfs::path &output_dir, const string &target_frame);
  void data_cb(const ImConstPtr &rgb, const ImConstPtr &thermal,
      const ImConstPtr &depth);
};

CloudTFVideoSynchronizer::CloudTFVideoSynchronizer(const bfs::path &output_dir,
    const string &target_frame):
    it(nh), count(-1), output_dir(output_dir),
    target_frame(target_frame), start(false), count_skip(5),
    sync(MySyncPolicy(20), rgb_sub, thermal_sub, depth_sub)
{
    rgb_sub.subscribe(it, "rgb_image", 20);
    thermal_sub.subscribe(it, "thermal_image", 20);
    depth_sub.subscribe(it, "depth_image", 20);
    tf_listener = boost::make_shared<tf2_ros::TransformListener>(tf_buffer);
    sync.registerCallback(boost::bind(&CloudTFVideoSynchronizer::data_cb, this,
        _1, _2, _3));
}

void CloudTFVideoSynchronizer::data_cb(const ImConstPtr &rgb,
    const ImConstPtr &thermal, const ImConstPtr &depth) {

    if (!start) {
        try {
            tf_buffer.lookupTransform("turntable_base", target_frame, Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
    }
    start = true;
    count++;

    if (count % count_skip != 0) return;
    int c = count / count_skip;

    // save images
    stringstream ss;
    bool done(true);
    ss << setfill('0') << setw(5) << c << "_rgb.png";
    bfs::path filename = output_dir / ss.str();
    done &= cv::imwrite(filename.string(), cv_bridge::toCvShare(rgb)->image);

    ss.str("");
    ss << setfill('0') << setw(5) << c << "_thermal.png";
    filename = output_dir / ss.str();
    done &= cv::imwrite(filename.string(), cv_bridge::toCvShare(thermal)->image);

    ss.str("");
    ss << setfill('0') << setw(5) << c << "_depth.png";
    filename = output_dir / ss.str();
    done &= cv::imwrite(filename.string(), cv_bridge::toCvShare(depth)->image);

    if (done) ROS_INFO_STREAM("Frame " << c);
}


bool create_dir(const bfs::path &p) {
    if (bfs::is_directory(p)) {
        cout << "Directory " << p.string() << " already exists" << endl;
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
    ros::init(argc, argv, "cloud_tf_sync_video");

    if (argc != 3) {
        cout << "Usage: ./" << argv[0] << " data_dir object_name" << endl;
        return -1;
    }
    bfs::path output_dir(argv[1]);
    output_dir /= argv[2];
    output_dir /= "sync_images";

    // create directories
    bool done = create_dir(output_dir);
    if (done) ROS_INFO("Created directories");
    else {
        ROS_ERROR("Could not create directories");
        return -1;
    }

    CloudTFVideoSynchronizer cloud_tf_sync(output_dir, "turntable_frame");
    ros::spin();

    return 0;
}