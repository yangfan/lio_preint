#include "Sync.h"

#include <glog/logging.h>

void Sync::config(const std::string &yaml_file) { converter.config(yaml_file); }

void Sync::add_imu(std::unique_ptr<sensor_msgs::msg::Imu> imu_msg) {
  const double timestamp = rclcpp::Time(imu_msg->header.stamp).seconds();
  if (timestamp < last_imu_time_) {
    LOG(WARNING) << "Wrong imu message in buffer. Clear the buffer.";
    imu_buffer_.clear();
  }
  const Eigen::Vector3d omega(imu_msg->angular_velocity.x,
                              imu_msg->angular_velocity.y,
                              imu_msg->angular_velocity.z);
  const Eigen::Vector3d acc(imu_msg->linear_acceleration.x,
                            imu_msg->linear_acceleration.y,
                            imu_msg->linear_acceleration.z);
  imu_buffer_.emplace_back(new IMU(timestamp, omega, acc));
  last_imu_time_ = timestamp;
}

void Sync::add_cloud(std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud_msg) {
  if (rclcpp::Time(cloud_msg->header.stamp).seconds() < last_cloud_time_) {
    LOG(WARNING) << "Wrong scan message in buffer. Clear the buffer.";
    cloud_buffer_.clear();
    time_buffer_.clear();
  }

  LidarPointCloudPtr cloud(new LidarPointCloud);
  converter.convert(*cloud_msg, cloud);

  if (cloud->empty()) {
    return;
  }
  cloud_buffer_.emplace_back(cloud);
  last_cloud_time_ = rclcpp::Time(cloud_msg->header.stamp).seconds();
  time_buffer_.emplace_back(last_cloud_time_);

  sync();
}

bool Sync::sync() {
  if (imu_buffer_.empty() || cloud_buffer_.empty()) {
    return false;
  }
  if (!cloud_imu_.scan) {
    cloud_imu_.scan = cloud_buffer_.front();
    cloud_imu_.scan_start_time = time_buffer_.front();
    cloud_imu_.scan_end_time =
        cloud_imu_.scan_start_time +
        cloud_imu_.scan->points.back().time / double(1000.0); // second
  }
  if (last_imu_time_ < cloud_imu_.scan_end_time) {
    // LOG(INFO) << "not enough imu readings.";
    return false;
  }
  cloud_buffer_.pop_front();
  time_buffer_.pop_front();

  while (!imu_buffer_.empty() &&
         imu_buffer_.front()->timestamp < cloud_imu_.scan_end_time) {
    cloud_imu_.imu_sequence.emplace_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  if (call_back_) {
    call_back_(cloud_imu_);
  } else {
    LOG(ERROR) << "No callback function asigned.";
  }

  cloud_imu_.scan = nullptr;
  cloud_imu_.imu_sequence.clear();
  cloud_imu_.scan_start_time = -1.0;
  cloud_imu_.scan_end_time = -1.0;
  return true;
}
