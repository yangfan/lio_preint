#pragma once

#include <memory>

#include <Eigen/Core>
#include <glog/logging.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

struct IMU {
  IMU() = default;
  IMU(const double time, const Eigen::Vector3d &omega, const Eigen::Vector3d &a)
      : timestamp(time), gyr(omega), acc(a) {}
  double timestamp{0.0};
  Eigen::Vector3d gyr{};
  Eigen::Vector3d acc{};
};
using IMUPtr = std::shared_ptr<IMU>;

struct IMUState {
  IMUState() = default;
  IMUState(const Eigen::Matrix<double, 18, 1> &vec)
      : pos(vec.head<3>()), vel(vec.segment<3>(3)),
        rot(Sophus::SO3d::exp(vec.segment<3>(6))), bias_g(vec.segment<3>(9)),
        bias_a(vec.segment<3>(12)), gravity(vec.tail<3>()) {}

  Eigen::Matrix<double, 18, 1> vec() const {
    Eigen::Matrix<double, 18, 1> res;
    res << pos, vel, rot.log(), bias_g, bias_a, gravity;
    return res;
  }
  Sophus::SE3d SE3() const { return Sophus::SE3d(rot, pos); }

  double timestamp{0.0};
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Sophus::SO3d rot{};
  Eigen::Vector3d bias_g = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity{0, 0, -9.8};
};

class IMUPreintegrator {
public:
  IMUPreintegrator() {
    reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0);
  }
  IMUPreintegrator(const Eigen::Vector3d bias_g, const Eigen::Vector3d bias_a,
                   const Eigen::Vector3d var_g, const Eigen::Vector3d var_a,
                   const double timestamp) {
    reset(bias_g, bias_a, var_g, var_a, timestamp);
  }
  void reset(const Eigen::Vector3d bias_g, const Eigen::Vector3d bias_a,
             const Eigen::Vector3d var_g, const Eigen::Vector3d var_a,
             const double timestamp) {
    measurement_ij_.rot = Sophus::SO3d();
    measurement_ij_.vel = Eigen::Vector3d::Zero();
    measurement_ij_.pos = Eigen::Vector3d::Zero();
    measurement_ij_.bias_g = bias_g;
    measurement_ij_.bias_a = bias_a;
    measurement_ij_.timestamp = timestamp;
    cov_noise_.diagonal() << var_g, var_a;
    cov_preint_.setZero();
    dR_dbg_.setZero();
    dv_dbg_.setZero();
    dv_dba_.setZero();
    dp_dbg_.setZero();
    dp_dba_.setZero();
  }
  void set_gravity(const Eigen::Vector3d gravity) {
    measurement_ij_.gravity = gravity;
  }
  Eigen::Vector3d get_gravity() const { return measurement_ij_.gravity; }
  void set_time(const double timestamp) {
    measurement_ij_.timestamp = timestamp;
  }

  bool integrate(const IMU &imu_data);
  // predict current state say j based on preintegration measurement between i
  // and j
  IMUState predict(const IMUState &start) const;
  // update preintegration measurement by incorporating bias update
  Sophus::SO3d get_rot(const Eigen::Vector3d bg);
  Eigen::Vector3d get_vel(const Eigen::Vector3d bg,
                          const Eigen::Vector3d ba) const;
  Eigen::Vector3d get_pos(const Eigen::Vector3d bg,
                          const Eigen::Vector3d ba) const;

  Eigen::Matrix3d dR_dbg() const { return dR_dbg_; }
  Eigen::Matrix3d dv_dbg() const { return dv_dbg_; }
  Eigen::Matrix3d dv_dba() const { return dv_dba_; }
  Eigen::Matrix3d dp_dbg() const { return dp_dbg_; }
  Eigen::Matrix3d dp_dba() const { return dp_dba_; }

  Eigen::Matrix3d Jrb(const Eigen::Vector3d bg) const {
    return Sophus::SO3d::leftJacobian(-dR_dbg_ * (bg - measurement_ij_.bias_g));
  }

  Eigen::Matrix<double, 9, 9> info() const { return cov_preint_.inverse(); }
  // for odom correction
  Eigen::Matrix<double, 9, 9> cov_rvp() const { return cov_preint_; }
  IMUState measurement() const { return measurement_ij_; }
  void update_dv(const Eigen::Vector3d delta_dv) {
    measurement_ij_.vel += delta_dv;
  }
  void update_cov_v(const Eigen::Matrix3d cov_v) {
    cov_preint_.block<3, 3>(3, 3) = cov_v;
  }

private:
  IMUState measurement_ij_{}; // delta state from time step i to current, say k
  Eigen::Matrix<double, 9, 9> cov_preint_ =
      Eigen::Matrix<double, 9, 9>::Zero(); // covariance of preintegration
                                           // noise, i.e., err R, v, p
  Eigen::Matrix<double, 6, 6> cov_noise_ =
      Eigen::Matrix<double, 6, 6>::Zero(); // covariance of imu measurement
                                           // noise, i.e., err acc, gyro
  // jacobian of bias that describes influence of change in bias estimate on imu
  // measurements.
  Eigen::Matrix3d dR_dbg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dv_dbg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dv_dba_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dp_dbg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dp_dba_ = Eigen::Matrix3d::Zero();

  // IMU prev_data_;
};
