#pragma once

#include "PointCloudConverter.h"
#include "imu/imu.h"

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <deque>
#include <functional>
#include <string>

class Sync {
public:
  struct DataGroup {
    double scan_start_time = -1.0;
    double scan_end_time = -1.0;
    LidarPointCloudPtr scan;
    std::deque<IMUPtr> imu_sequence;
  };
  using CallBack = std::function<void(DataGroup &)>;

  Sync() = default;
  Sync(CallBack call_back) : call_back_(call_back) {}

  void config(const std::string &yaml_file);

  void add_imu(std::unique_ptr<sensor_msgs::msg::Imu> imu_msg);
  void add_cloud(std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud_msg);

private:
  std::deque<IMUPtr> imu_buffer_;
  std::deque<LidarPointCloudPtr> cloud_buffer_;
  std::deque<double> time_buffer_;
  PointCloudConverter converter;
  DataGroup cloud_imu_;
  CallBack call_back_;

  double last_cloud_time_ = 0.0;
  double last_imu_time_ = 0.0;
  bool imu_has_gravity_ = true;

  bool sync();
};