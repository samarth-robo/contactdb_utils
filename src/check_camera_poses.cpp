//
// Created by samarth on 3/18/18.
//
#include "deepgrasp_utils/Vis.h"
#include "deepgrasp_utils/common.h"

#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <cstdlib>

using namespace std;
namespace fs = boost::filesystem;
using namespace pcl;

typedef PointXYZ PointT;
typedef PointCloud<PointT> PointCloudT;

int main(int argc, char **argv) {
  if (argc != 4) {
    cout << "Usage: ./" << argv[0] << " data_dir p_id object_name" << endl;
    return -1;
  }

  fs::path base_dir = fs::path(argv[1]) / argv[2] / argv[3];

  // read the object
  PointCloudT::Ptr object = boost::make_shared<PointCloudT>();
  string object_name(argv[3]);
  object_name = object_name.substr(0, object_name.find_first_of('-'));
  fs::path filename = fs::path(std::getenv("HOME")) / "deepgrasp_data" /
      "models" / (object_name + ".ply");
  sample_mesh<PointT>(filename.string(), object);
  PCL_INFO("Read object %s with size %d\n", filename.string().c_str(),
      object->size());

  // create Vis object
  Vis vis("check camera poses");

  // collect pose paths
  vector<fs::path> pose_paths;
  for (fs::directory_iterator it(base_dir/"poses");
      it!=fs::directory_iterator(); it++) {
    if (!fs::is_regular_file(it->status())) continue;
    if (it->path().filename().string().find("object_pose") == string::npos)
      continue;
    pose_paths.push_back(it->path());
  }
  sort(pose_paths.begin(), pose_paths.end());

  for (auto const &p: pose_paths) {
    // read scene
    PointCloudT::Ptr scene = boost::make_shared<PointCloudT>();
    string scene_name = p.stem().string();
    scene_name = scene_name.substr(scene_name.find_last_of("_")+1);
    stringstream ss;
    ss << scene_name << ".pcd";
    filename = base_dir / "pointclouds" / ss.str();
    io::loadPCDFile(filename.string(), *scene);
    PCL_INFO("Read scene %s with size %d x %d\n", filename.string().c_str(),
        scene->width, scene->height);

    // read pose
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    ifstream f(p.string());
    if (!f.is_open()) {
      PCL_ERROR("Could not open %s for reading\n", filename.string().c_str());
      return -1;
    }
    string line;
    getline(f, line);
    getline(f, line); {istringstream ss(line); ss >> T(0, 3);}
    getline(f, line); {istringstream ss(line); ss >> T(1, 3);}
    getline(f, line); {istringstream ss(line); ss >> T(2, 3);}
    getline(f, line);
    getline(f, line);
    getline(f, line); {istringstream ss(line); ss >> T(0, 0) >> T(0, 1) >> T(0, 2);}
    getline(f, line); {istringstream ss(line); ss >> T(1, 0) >> T(1, 1) >> T(1, 2);}
    getline(f, line); {istringstream ss(line); ss >> T(2, 0) >> T(2, 1) >> T(2, 2);}
    f.close();
    cout << "Read Pose" << endl << T << endl;

    // transform object
    PointCloudT::Ptr obj_t = boost::make_shared<PointCloudT>();
    transformPointCloud(*object, *obj_t, T);

    // show
    vis.addPointCloud<PointT>(scene, {0, 1, 0});
    vis.addPointCloud<PointT>(obj_t, {1, 0, 0});
    // vis.setCameraParams(0.00364115, 3.64115,
    //     -0.0200831, 0.0568331, 0.763494,
    //     0.0183852, -0.228605, -0.55524,
    //     -0.0178499, -0.977319, 0.211019,
    //     30);
    vis.show(true);
    vis.removeAllPointClouds();
  }

  return 0;
}