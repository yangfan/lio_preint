#include "lio/LioPreint.h"
#include "tools/BagIO.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

DEFINE_string(bag_file,
              // "/home/fan/ssd/Projects/ros2_ws/data/bags/ULHK/ros2_cloud2",
              "/home/fan/ssd/Projects/ros2_ws/data/bags/NCLT/ros2_0110",
              "path to ros2bag file");
DEFINE_string(
    config_file,
    // "/home/fan/ssd/Projects/ros2_ws/src/lio_preint/config/velodyne_ulhk.yaml",
    "/home/fan/ssd/Projects/ros2_ws/src/lio_preint/config/velodyne_nclt.yaml",
    "path of configuration file");
// DEFINE_string(topic_name, "",
// DEFINE_string(cloud_topic_name, "/velodyne_points_0",
//               "topic name of pointcloud data");
DEFINE_string(cloud_topic_name, "points_raw", "topic name of pointcloud data");

// DEFINE_string(imu_topic_name, "/imu/data", "topic name of imu data");
DEFINE_string(imu_topic_name, "imu_raw", "topic name of imu data");

DEFINE_string(data_path,
              "/home/fan/ssd/Projects/ros2_ws/src/lio_preint/data/output/",
              "Path to save map file");

TEST(IESKF, LIO) {
  BagIO bag_io(FLAGS_bag_file);
  LioPreint preint;
  preint.config(FLAGS_config_file);

  bag_io
      .AddPointCloudHandle(
          FLAGS_cloud_topic_name,
          [&](std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud_msg) {
            preint.add_scan(std::move(cloud_msg));
            return true;
          })
      .AddIMUHandle(FLAGS_imu_topic_name,
                    [&](std::unique_ptr<sensor_msgs::msg::Imu> imu_msg) {
                      preint.add_imu(std::move(imu_msg));
                      return true;
                    })
      .Process();
  preint.save_map(FLAGS_data_path + "nclt_preint.pcd");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  google::ParseCommandLineFlags(&argc, &argv, true);

  return RUN_ALL_TESTS();
}