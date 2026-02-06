#include "imu/ImuInitializer.h"
#include <glog/logging.h>

void ImuInitializer::config(const std::string &yaml_file) {
  auto node = YAML::LoadFile(yaml_file);
  params_.imu_has_gravity = node["imu_has_gravity"].as<bool>();
}

bool ImuInitializer::addImu(const IMU &imu_data) {
  if (initialized_) {
    return true;
  }
  while (imu_deque_.size() >= params_.deque_cap) {
    imu_deque_.pop_front();
  }
  imu_deque_.push_back(imu_data);

  if (imu_deque_.size() > params_.deque_min &&
      imu_data.timestamp - imu_deque_.front().timestamp >=
          params_.measure_time) {
    initialized_ = initialize();
  }
  return initialized_;
}

bool ImuInitializer::initialize() {
  LOG(INFO) << "deque size: " << imu_deque_.size();
  mean_variance(imu_deque_, init_bias_g_, init_var_g_,
                [](const auto &imu) { return imu.gyr; });
  LOG(INFO) << "Initial mean gyr bias: " << init_bias_g_.transpose();
  LOG(INFO) << "Initial mean gyr varianc: " << init_var_g_.transpose();

  Eigen::Vector3d raw_bias_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d raw_var_a = Eigen::Vector3d::Zero();
  mean_variance(imu_deque_, raw_bias_a, raw_var_a,
                [](const auto &imu) { return imu.acc; });
  LOG(INFO) << "raw bias a: " << raw_bias_a.transpose()
            << ", norm: " << raw_bias_a.norm();

  init_grav_ = -(raw_bias_a.normalized()) * params_.kgravity;

  if (!params_.imu_has_gravity) {
    init_bias_a_ = raw_bias_a;
    init_var_a_ = raw_var_a;
  } else {
    mean_variance(imu_deque_, init_bias_a_, init_var_a_,
                  [this](const auto &imu) { return (imu.acc + init_grav_); });
  }
  LOG(INFO) << "Initial mean acc bias: " << init_bias_a_.transpose();
  LOG(INFO) << "Initial mean acc varianc: " << init_var_a_.transpose();

  LOG(INFO) << "Initial gravity: " << init_grav_.transpose();
  LOG(INFO) << std::fixed << "Initial timestamp: " << timestamp_;

  timestamp_ = imu_deque_.back().timestamp;

  return true;
}

template <typename GetterType>
void ImuInitializer::mean_variance(const std::deque<IMU> &data,
                                   Eigen::Vector3d &mean,
                                   Eigen::Vector3d &variance,
                                   GetterType getter) {
  mean =
      std::accumulate(data.begin(), data.end(), Eigen::Vector3d::Zero().eval(),
                      [&getter](const Eigen::Vector3d &sum, const IMU &imu)
                          -> Eigen::Vector3d { return sum + getter(imu); }) /
      data.size();
  LOG(INFO) << "mean: " << mean.transpose();
  variance =
      std::accumulate(data.begin(), data.end(), Eigen::Vector3d::Zero().eval(),
                      [&getter, &mean](const Eigen::Vector3d &sum,
                                       const IMU &imu) -> Eigen::Vector3d {
                        return sum + (getter(imu) - mean).cwiseAbs2().eval();
                      }) /
      (data.size() - 1);
}

void ImuInitializer::print() const {
  LOG(INFO) << "ba: " << init_bias_a_.transpose()
            << ", bg: " << init_bias_g_.transpose();
  LOG(INFO) << "va: " << init_var_a_.transpose()
            << ", vg: " << init_var_g_.transpose();
  LOG(INFO) << "g: " << init_grav_.transpose();
}