//
// Created by samarth on 4/5/17.
//

#ifndef PROJECT_VIS_H
#define PROJECT_VIS_H

#include <string>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

class Vis {
 private:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  unsigned int count;
  bool shown;
  std::string get_next_id(std::string word);
 public:
  Vis(std::string window_name = std::string("cloud"));
  bool addPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                     double pt_size = 1.0);
  bool addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                     double pt_size = 1.0);
  template <typename PointT>
  bool addPointCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                     std::vector<double> colors, double pt_size = 1.0);
  template <typename PointT, typename PointNT>
  bool addPointCloudNormals(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                            typename pcl::PointCloud<PointNT>::ConstPtr normals);
  template <typename PointT>
  bool addLine(PointT p0, PointT p1,
               std::vector<double> colors = {1.0, 1.0, 0.0});
  bool addCube(pcl::PointXYZ min_pt, pcl::PointXYZ max_pt,
               std::vector<double> colors = {1.0, 1.0, 1.0});
  template<typename PointT>
  bool addSphere(PointT p, double radius = 1.0,
      std::vector<double> colors = {1, 1, 1});
  bool addText3D(std::string text, pcl::PointXYZ p,
                 std::vector<double> text_colors = {0.0, 1.0, 1.0});
  template <typename PointT>
  bool addCorrespondences(typename pcl::PointCloud<PointT>::ConstPtr source,
                          typename pcl::PointCloud<PointT>::ConstPtr target,
                          pcl::CorrespondencesConstPtr const &corrs, int skip = 20);
  template <typename PointT>
  bool addCorrespondences(typename pcl::PointCloud<PointT>::ConstPtr source,
                          typename pcl::PointCloud<PointT>::ConstPtr target,
                          std::vector<int> const &corrs, int skip = 20);
  void show(bool block = true);
  void removeAllPointClouds() {viewer->removeAllPointClouds();}
  void setCameraParams(float clip_near, float clip_far,
      float foc_x, float foc_y, float foc_z,
      float pos_x, float pos_y, float pos_z,
      float up_x,  float up_y,  float up_z,
      float view_angle);
};

#endif //PROJECT_VIS_H
