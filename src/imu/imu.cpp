#include "imu/imu.h"
#include <glog/logging.h>

bool IMUIntegrator::integrate(const IMU &imu_data) {
  const double dt = imu_data.timestamp - state_.timestamp;
  if (dt > 0 && dt < 0.1) {
    const Eigen::Vector3d acc =
        state_.rot * (imu_data.acc - state_.bias_a) + state_.gravity;
    state_.pos += state_.vel * dt + 0.5 * acc * dt * dt;
    state_.vel += acc * dt;
    state_.rot *= Sophus::SO3d::exp((imu_data.gyr - state_.bias_g) * dt);
  }

  state_.timestamp = imu_data.timestamp;
  // assumming bias and gravity remain the same
  return true;
}

bool IMUPreintegrator::integrate(const IMU &imu_data) {
  const double dt =
      imu_data.timestamp -
      measurement_ij_.timestamp; // time interval between current and previous
                                 // (frame) integration
  if (dt <= 0) {
    LOG(WARNING) << "IMU data is obsolete.";
    return false;
  }
  const double dt2 = dt * dt;

  IMUState next_measurement = measurement_ij_;
  next_measurement.timestamp = imu_data.timestamp;
  Eigen::Vector3d domega = (imu_data.gyr - measurement_ij_.bias_g) * dt;
  Sophus::SO3d domega_R = Sophus::SO3d::exp(domega);
  Eigen::Vector3d dacc = imu_data.acc - measurement_ij_.bias_a;
  Eigen::Vector3d dacc_i = measurement_ij_.rot * dacc;

  // 1. update preintegration mesaurement
  next_measurement.rot = measurement_ij_.rot * domega_R;
  next_measurement.vel = measurement_ij_.vel + dacc_i * dt;
  next_measurement.pos =
      measurement_ij_.pos + measurement_ij_.vel * dt + 0.5 * dacc_i * dt2;

  Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
  A.block<3, 3>(0, 0) = domega_R.matrix().transpose();
  Eigen::Matrix3d dR = measurement_ij_.rot.matrix() * Sophus::SO3d::hat(dacc);
  A.block<3, 3>(3, 0) = -dR * dt;
  A.block<3, 3>(6, 0) = -0.5 * dR * dt2;
  A.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;

  Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();
  Eigen::Matrix3d Jr = Sophus::SO3d::leftJacobian(-domega);
  B.block<3, 3>(0, 0) = Jr * dt;
  B.block<3, 3>(3, 3) = measurement_ij_.rot.matrix() * dt;
  B.block<3, 3>(6, 3) = 0.5 * measurement_ij_.rot.matrix() * dt2;

  // 2. update noise for each measurement
  cov_preint_ =
      A * cov_preint_.eval() * A.transpose() + B * cov_noise_ * B.transpose();

  // 3. update jocobian of bias for bias update
  dp_dba_ =
      dp_dba_.eval() + dv_dba_ * dt - 0.5 * measurement_ij_.rot.matrix() * dt2;
  dp_dbg_ = dp_dbg_.eval() + dv_dbg_ * dt - 0.5 * dR * dR_dbg_ * dt2;

  dv_dba_ = dv_dba_.eval() - measurement_ij_.rot.matrix() * dt;
  dv_dbg_ = dv_dbg_.eval() - dR * dR_dbg_ * dt;

  dR_dbg_ = domega_R.matrix().transpose() * dR_dbg_.eval() - Jr * dt;

  measurement_ij_ = next_measurement;
  return true;
}

IMUState IMUPreintegrator::predict(const IMUState &start) const {
  IMUState curr;
  if (start.timestamp > measurement_ij_.timestamp) {
    LOG(WARNING) << "Fail to predict current state. The time stamp of starting "
                    "state is after the measurement.";
    return curr;
  }
  double t_ij = measurement_ij_.timestamp - start.timestamp;
  curr.rot = start.rot * measurement_ij_.rot;
  curr.vel = start.rot * measurement_ij_.vel + start.vel +
             measurement_ij_.gravity * t_ij;
  curr.pos = start.rot * measurement_ij_.pos + start.pos + start.vel * t_ij +
             0.5 * measurement_ij_.gravity * t_ij * t_ij;
  curr.bias_g = measurement_ij_.bias_g;
  curr.bias_a = measurement_ij_.bias_a;
  curr.gravity = measurement_ij_.gravity;
  curr.timestamp = measurement_ij_.timestamp;

  return curr;
}

Sophus::SO3d IMUPreintegrator::get_rot(const Eigen::Vector3d bg) {
  return measurement_ij_.rot *
         Sophus::SO3d::exp(dR_dbg_ * (bg - measurement_ij_.bias_g));
}

Eigen::Vector3d IMUPreintegrator::get_vel(const Eigen::Vector3d bg,
                                          const Eigen::Vector3d ba) const {
  return measurement_ij_.vel + dv_dbg_ * (bg - measurement_ij_.bias_g) +
         dv_dba_ * (ba - measurement_ij_.bias_a);
}

Eigen::Vector3d IMUPreintegrator::get_pos(const Eigen::Vector3d bg,
                                          const Eigen::Vector3d ba) const {
  return measurement_ij_.pos + dp_dbg_ * (bg - measurement_ij_.bias_g) +
         dp_dba_ * (ba - measurement_ij_.bias_a);
}
