#pragma once

#include "imu.h"

#include <deque>
#include <string>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

class ImuInitializer {
public:
  void config(const std::string &yaml_file);

  bool addImu(const IMU &imu_data);
  bool initialize();
  bool success() const { return initialized_; }

  struct Params {
    size_t deque_cap = 2000;
    size_t deque_min = 10;
    double measure_time = 10.0;
    double kgravity = 9.81;
    bool imu_has_gravity = true;
  };

  Eigen::Vector3d bias_a() const { return init_bias_a_; }
  Eigen::Vector3d var_a() const { return init_var_a_; }

  Eigen::Vector3d bias_g() const { return init_bias_g_; }
  Eigen::Vector3d var_g() const { return init_var_g_; }

  Eigen::Vector3d gravity() const { return init_grav_; }
  double timestamp() const { return timestamp_; }

  void print() const;

private:
  Eigen::Vector3d init_bias_g_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_bias_a_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_var_g_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_var_a_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_grav_ = Eigen::Vector3d::Zero();

  std::deque<IMU> imu_deque_;

  bool initialized_ = false;
  double timestamp_ = 0;

  Params params_;

  template <typename GetterType>
  void mean_variance(const std::deque<IMU> &data, Eigen::Vector3d &mean,
                     Eigen::Vector3d &variance, GetterType getter);
};