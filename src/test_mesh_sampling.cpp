//
// Created by samarth on 5/1/17.
//

#include <deepgrasp_utils/common.h>
#include <deepgrasp_utils/Vis.h>

using namespace std;
using namespace pcl;

int main(int argc, char **argv) {
  if (argc != 2) {
    console::print_info("Usage: %s object.ply", argv[0]);
    return -1;
  }

  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
  sample_mesh<PointXYZ>(argv[1], cloud, 0.01);

  Vis vis("sampled cloud");
  vis.addPointCloud(cloud);
  vis.show();

  return 0;
}
