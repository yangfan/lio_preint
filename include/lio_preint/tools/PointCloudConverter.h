#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "LidarPointType.h"

#include <string>

class PointCloudConverter {
public:
  void convert(const sensor_msgs::msg::PointCloud2 &msg,
               LidarPointCloudPtr point_cloud);
  void config(const std::string &yaml_file);

private:
  int point_filter_num_ = 1;
  int num_scans_ = 6;
  float time_scale_ = 1e-3;

  void velodyne_handler(const sensor_msgs::msg::PointCloud2 &msg,
                        LidarPointCloudPtr point_cloud);
};