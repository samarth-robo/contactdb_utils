//
// Created by samarth on 5/1/17.
//

#ifndef PROJECT_COMMON_H
#define PROJECT_COMMON_H

#include <string>
#include <pcl/point_cloud.h>

template <typename PointT>
void sample_mesh(std::string ply_filename, typename pcl::PointCloud<PointT>::Ptr &cloud_out,
                 int n_samples=10000, bool process_normals=false);

#endif //PROJECT_COMMON_H
