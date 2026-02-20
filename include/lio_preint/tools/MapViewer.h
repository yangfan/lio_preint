#pragma once

#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sophus/se3.hpp>

#include <string>

class MapViewer {
public:
  MapViewer() : MapViewer("Map Viewer", 1.0f) {}
  explicit MapViewer(const std::string &win_name, const float lf_sz)
      : map_(new pcl::PointCloud<pcl::PointXYZI>), leaf_size_(lf_sz) {
    voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    visualizer_.setWindowName(win_name);
    // visualizer_.setSize(1920, 1080);
    visualizer_.getRenderWindow()->GlobalWarningDisplayOff();
    visualizer_.addCoordinateSystem(10, "world");
  }
  void set_leaf_size(const float sz) { leaf_size_ = sz; }
  void clear() { map_->clear(); }
  void reset(const float lf_sz) {
    map_->clear();
    leaf_size_ = lf_sz;
    voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  }

  bool add_pointcloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointcloud,
                      const Sophus::SE3d &body_pose);
  bool save_map(const std::string &file) const;

  void spin() { visualizer_.spin(); }

private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_;
  pcl::visualization::PCLVisualizer visualizer_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
  float leaf_size_ = 1.0f;
  size_t max_pt_num_ = 600000;
  bool initialized_ = false;
  size_t kf_num_ = 0;
};