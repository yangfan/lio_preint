#include "MapViewer.h"
#include <glog/logging.h>

bool MapViewer::save_map(const std::string &file) const {
  std::ofstream ofs(file);
  if (map_->empty() || !ofs.is_open()) {
    return false;
  }
  pcl::io::savePCDFile(file, *map_);
  return true;
}

bool MapViewer::add_pointcloud(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointcloud,
    const Sophus::SE3d &body_pose) {
  if (pointcloud->empty()) {
    return false;
  }
  voxel_filter_.setInputCloud(pointcloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
  voxel_filter_.filter(*tmp);

  *map_ += *tmp;
  voxel_filter_.setInputCloud(map_);
  voxel_filter_.filter(*map_);

  visualizer_.removePointCloud("map");
  visualizer_.removeCoordinateSystem("vehicle");

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
      field_color(map_, "z");
  visualizer_.addPointCloud(map_, field_color, "map");

  Eigen::Affine3f T;
  T.matrix() = body_pose.matrix().cast<float>();
  visualizer_.addCoordinateSystem(5, T, "vehicle");
  visualizer_.spinOnce(1);

  if (map_->size() > max_pt_num_) {
    leaf_size_ *= 1.2;
    voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  }

  return true;
}
