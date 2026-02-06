#include "PointCloudConverter.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

void PointCloudConverter::convert(const sensor_msgs::msg::PointCloud2 &msg,
                                  LidarPointCloudPtr lidar_pointcloud) {
  velodyne_handler(msg, lidar_pointcloud);
}

void PointCloudConverter::config(const std::string &yaml_file) {

  LOG(INFO) << "Configuring converter.";
  auto yaml_node = YAML::LoadFile(yaml_file);

  point_filter_num_ = yaml_node["point_filter_num"].as<int>();
  LOG(INFO) << "point filter num: " << point_filter_num_;
  num_scans_ = yaml_node["preprocess"]["scan_line"].as<int>();
  LOG(INFO) << "num scans: " << num_scans_;
  time_scale_ = yaml_node["preprocess"]["time_scale"].as<double>();
  LOG(INFO) << "time scale: " << time_scale_;
}

void PointCloudConverter::velodyne_handler(
    const sensor_msgs::msg::PointCloud2 &msg,
    LidarPointCloudPtr lidar_pointcloud) {

  lidar_pointcloud->clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(msg, pl_orig);

  const int plsize = pl_orig.points.size();
  lidar_pointcloud->points.reserve(plsize);

  double omega_l = 3.61; // scan angular velocity, not accurate enough
  std::vector<bool> is_first(num_scans_, true);
  std::vector<double> yaw_fp(num_scans_, 0.0);   // yaw of first scan point
  std::vector<float> yaw_last(num_scans_, 0.0);  // yaw of last scan point
  std::vector<float> time_last(num_scans_, 0.0); // last offset time

  bool given_offset_time = pl_orig.points[plsize - 1].time > 0 ? true : false;

  for (int i = 0; i < plsize; i++) {
    LidarPointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.time = pl_orig.points[i].time * time_scale_; // ms

    if (added_pt.getVector3fMap().norm() < 4.0) {
      continue;
    }

    if (!given_offset_time) {
      int layer = pl_orig.points[i].ring;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

      if (is_first[layer]) {
        yaw_fp[layer] = yaw_angle;
        is_first[layer] = false;
        added_pt.time = 0.0;
        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.time;
        continue;
      }

      // compute offset time
      if (yaw_angle <= yaw_fp[layer]) {
        added_pt.time = (yaw_fp[layer] - yaw_angle) / omega_l;
      } else {
        added_pt.time = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
      }

      if (added_pt.time < time_last[layer]) {
        added_pt.time += 360.0 / omega_l;
      }

      yaw_last[layer] = yaw_angle;
      time_last[layer] = added_pt.time;
    }

    if (i % point_filter_num_ == 0) {
      if (added_pt.time > 150) {
        LOG(WARNING) << "Long scan time: " << added_pt.time << " ms.";
      }
      lidar_pointcloud->points.push_back(added_pt);
    }
  }
  lidar_pointcloud->height = 1;
  lidar_pointcloud->width = lidar_pointcloud->size();
}
