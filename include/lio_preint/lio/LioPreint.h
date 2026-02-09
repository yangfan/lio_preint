#pragma once

#include "g2o_types/types.h"
#include "imu/ImuInitializer.h"
#include "imu/imu.h"
#include "matching/NDT_INC.h"
#include "tools/MapViewer.h"
#include "tools/Sync.h"

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

class LioPreint {
public:
  struct Params {
    int iterations = 20;

    double min_kf_dist = 0.3;
    double min_kf_deg = 5.0;
    bool viewer_on = true;

    double sigma_ndt_pos = 0.1;
    double sigma_ndt_ang = 2.0 * M_PI / 180.0;

    double sigma_ba = 1e-2;
    double sigma_bg = 1e-2;
  };

  bool config(const std::string &yaml_file);
  void add_imu(std::unique_ptr<sensor_msgs::msg::Imu> imu_msg);
  void add_scan(std::unique_ptr<sensor_msgs::msg::PointCloud2> scan_msg);

  void save_map(const std::string &map_file);

private:
  ImuInitializer imu_initializer_;
  Sync lidar_imu_sync_;
  NDT_INC ndt_inc_;
  bool ndt_initialized = false;
  std::unique_ptr<MapViewer> viewer_;

  IMUPreintegrator preint_;
  bool preint_initialized_ = false;

  Sync::DataGroup lidar_imu_;
  std::vector<IMUState> states_;
  Params params_;
  double deg2rad = M_PI / 180.0;

  Sophus::SE3d T_IL_;
  Sophus::SE3d last_kf_pose_;
  Sophus::SE3d aligned_pose_;

  // state of end of last imu integration interval
  IMUState last_state_;
  IMUState cur_state_;

  Eigen::Matrix<double, 3, 3> bg_info_ =
      Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 3, 3> ba_info_ =
      Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 6, 6> ndt_info_ =
      Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 15, 15> prior_info_ =
      Eigen::Matrix<double, 15, 15>::Identity();

  void process_sync_data(const Sync::DataGroup &lidar_imu);

  bool initialize_imu(const Sync::DataGroup &lidar_imu);

  void build_info();

  void predict();
  void undistort();
  void correct();
  void optimize();

  void normalize_vel();

  bool is_keyframe(const Sophus::SE3d &pose) const;

  Sophus::SE3d interpolation(const double ratio, const IMUState &state0,
                             const IMUState &state1) const;
  Sophus::SE3d integrate_imu(const IMUState &state, const IMUPtr &imu_measure,
                             const double dt) const;
  pcl::PointCloud<pcl::PointXYZI>::Ptr desampling(const double leaf_sz);

  void marginalize(const Eigen::Matrix<double, 30, 30> &H);

  template <typename S>
  Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<S> &v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
  }

  template <typename S>
  Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<S> &v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
  }
};