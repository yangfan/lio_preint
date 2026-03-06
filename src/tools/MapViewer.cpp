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

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
      field_color(map_, "z");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> scan_color(
      map_, 255, 255, 255);

  Eigen::Affine3f T;
  T.matrix() = body_pose.matrix().cast<float>();

  if (kf_num_++ % 10 == 0) {

    const pcl::PointXYZ astart(body_pose.translation().x(),
                               body_pose.translation().y(),
                               body_pose.translation().z());
    const Eigen::Vector3d arrow_end =
        body_pose * Eigen::Vector3d(0.0, -1.0, 0.0);
    const pcl::PointXYZ aend(arrow_end.x(), arrow_end.y(), arrow_end.z());

    visualizer_.addArrow(aend, astart, 255.0, 0, 0, false,
                         std::to_string(kf_num_));
  }

  if (!initialized_) {
    visualizer_.addPointCloud(map_, field_color, "map");
    visualizer_.addPointCloud(pointcloud, scan_color, "scan");
    visualizer_.addCoordinateSystem(5, T, "vehicle");
    initialized_ = true;
  } else {
    visualizer_.updatePointCloud(map_, field_color, "map");
    visualizer_.updatePointCloud(pointcloud, scan_color, "scan");
    visualizer_.updateCoordinateSystemPose("vehicle", T);
  }

  visualizer_.spinOnce(1);

  if (map_->size() > max_pt_num_) {
    leaf_size_ *= 1.2;
    voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  }

  return true;
}
