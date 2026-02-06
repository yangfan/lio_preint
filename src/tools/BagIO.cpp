#include "BagIO.h"
// #include <glog/logging.h>

BagIO &BagIO::AddHandle(const std::string &topic_name,
                        ProcessFunc process_func) {
  process_funcs_.emplace(topic_name, process_func);
  return *this;
}

BagIO &BagIO::AddPointCloudHandle(const std::string &topic_name,
                                  PointCloud2Handle func) {
  return AddHandle(
      topic_name,
      [&topic_name,
       func](rosbag2_storage::SerializedBagMessageConstSharedPtr msg) -> bool {
        if (msg->topic_name != topic_name) {
          return false;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        sensor_msgs::msg::PointCloud2::UniquePtr pointcloud_msg =
            std::make_unique<sensor_msgs::msg::PointCloud2>();
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
        serializer.deserialize_message(&serialized_msg, pointcloud_msg.get());
        if (!pointcloud_msg) {
          return false;
        }
        return func(std::move(pointcloud_msg));
      });
}

BagIO &BagIO::AddIMUHandle(const std::string &topic_name, IMUHandle func) {
  return AddHandle(
      topic_name,
      [&topic_name,
       &func](rosbag2_storage::SerializedBagMessageConstSharedPtr msg) -> bool {
        if (msg->topic_name != topic_name) {
          return false;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        sensor_msgs::msg::Imu::UniquePtr imu_msg =
            std::make_unique<sensor_msgs::msg::Imu>();
        rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
        serializer.deserialize_message(&serialized_msg, imu_msg.get());
        if (!imu_msg) {
          return false;
        }
        return func(std::move(imu_msg));
      });
}

void BagIO::Process() {
  rosbag2_storage::StorageOptions options;
  options.uri = bag_file_;
  std::unique_ptr<rosbag2_cpp::Reader> reader =
      rosbag2_transport::ReaderWriterFactory::make_reader(options);
  reader->open(options);

  while (reader->has_next()) {
    rosbag2_storage::SerializedBagMessageConstSharedPtr msg =
        reader->read_next();
    auto func_it = process_funcs_.find(msg->topic_name);
    // LOG(INFO) << "topic name: " << msg->topic_name;
    if (func_it != process_funcs_.end()) {
      func_it->second(msg);
    }
  }
}