#include "types.h"
#include <glog/logging.h>

void VertexVec3::oplusImpl(const double *update) {
  _estimate += Eigen::Map<const Eigen::Vector3d>(update);
}

void EdgePreint::computeError() {
  Sophus::SO3d rot_i = static_cast<VertexSO3 *>(_vertices[0])->estimate();
  Sophus::SO3d rot_j = static_cast<VertexSO3 *>(_vertices[1])->estimate();

  const Eigen::Vector3d vel_i =
      static_cast<VertexVel *>(_vertices[2])->estimate();
  const Eigen::Vector3d vel_j =
      static_cast<VertexVel *>(_vertices[3])->estimate();

  const Eigen::Vector3d pos_i =
      static_cast<VertexPos *>(_vertices[4])->estimate();
  const Eigen::Vector3d pos_j =
      static_cast<VertexPos *>(_vertices[5])->estimate();

  const Eigen::Vector3d bias_ai =
      static_cast<VertexBiasA *>(_vertices[6])->estimate();
  const Eigen::Vector3d bias_gi =
      static_cast<VertexBiasG *>(_vertices[7])->estimate();
  const Eigen::Vector3d gravity = preintegrator_->get_gravity();

  Sophus::SO3d delta_rot = preintegrator_->get_rot(bias_gi);
  const Eigen::Vector3d delta_vel = preintegrator_->get_vel(bias_gi, bias_ai);
  const Eigen::Vector3d delta_pos = preintegrator_->get_pos(bias_gi, bias_ai);

  const Eigen::Vector3d err_rot =
      (delta_rot.inverse() * rot_i.inverse() * rot_j).log();
  const Eigen::Vector3d err_vel =
      rot_i.inverse() * (vel_j - vel_i - gravity * t_ij_) - delta_vel;
  const Eigen::Vector3d err_pos =
      rot_i.inverse() *
          (pos_j - pos_i - vel_i * t_ij_ - 0.5 * gravity * t_ij_ * t_ij_) -
      delta_pos;
  // if (err_pos.norm() > 1.0) {
  //   LOG(INFO) << "delta_pos: " << delta_pos.transpose();
  //   LOG(INFO) << "obs pose: "
  //             << (rot_i.inverse() * (pos_j - pos_i - vel_i * t_ij_ -
  //                                    0.5 * gravity * t_ij_ * t_ij_))
  //                    .transpose();
  //   LOG(INFO) << "ba: " << bias_ai.transpose();
  //   LOG(INFO) << "bg: " << bias_gi.transpose();
  //   LOG(INFO) << "err pos norm: " << err_pos.norm();
  // }
  // if (err_vel.norm() > 1.0) {
  //   LOG(INFO) << "ba: " << bias_ai.transpose();
  //   LOG(INFO) << "bg: " << bias_gi.transpose();
  //   LOG(INFO) << "err vel norm: " << err_vel.norm();
  // }

  _error << err_rot, err_vel, err_pos;
}
void EdgePreint::linearizeOplus() {
  auto rot_i = static_cast<VertexSO3 *>(_vertices[0])->estimate();
  auto rot_j = static_cast<VertexSO3 *>(_vertices[1])->estimate();

  auto vel_i = static_cast<VertexVel *>(_vertices[2])->estimate();
  auto vel_j = static_cast<VertexVel *>(_vertices[3])->estimate();

  auto pos_i = static_cast<VertexPos *>(_vertices[4])->estimate();
  auto pos_j = static_cast<VertexPos *>(_vertices[5])->estimate();

  auto bias_ai = static_cast<VertexBiasA *>(_vertices[6])->estimate();
  auto bias_gi = static_cast<VertexBiasG *>(_vertices[7])->estimate();
  const Eigen::Vector3d gravity = preintegrator_->get_gravity();

  Sophus::SO3d err_SO3 =
      preintegrator_->get_rot(bias_gi).inverse() * rot_i.inverse() * rot_j;
  const Eigen::Matrix3d Jr_inv =
      Sophus::SO3d::leftJacobianInverse(-err_SO3.log());

  _jacobianOplus[0].block<3, 3>(0, 0) =
      -Jr_inv * (rot_j.inverse() * rot_i).matrix();
  _jacobianOplus[0].block<3, 3>(3, 0) =
      Sophus::SO3d::hat(rot_i.inverse() * (vel_j - vel_i - gravity * t_ij_));
  _jacobianOplus[0].block<3, 3>(6, 0) =
      Sophus::SO3d::hat(rot_i.inverse() * (pos_j - pos_i - vel_i * t_ij_ -
                                           0.5 * gravity * t_ij_ * t_ij_));

  _jacobianOplus[1].setZero();
  _jacobianOplus[1].block<3, 3>(0, 0) = Jr_inv;

  _jacobianOplus[2].setZero();
  _jacobianOplus[2].block<3, 3>(3, 0) = -rot_i.matrix().transpose();
  _jacobianOplus[2].block<3, 3>(6, 0) = -rot_i.matrix().transpose() * t_ij_;

  _jacobianOplus[3].setZero();
  _jacobianOplus[3].block<3, 3>(3, 0) = rot_i.matrix().transpose();

  _jacobianOplus[4].setZero();
  _jacobianOplus[4].block<3, 3>(6, 0) = -rot_i.matrix().transpose();

  _jacobianOplus[5].setZero();
  _jacobianOplus[5].block<3, 3>(6, 0) = rot_i.matrix().transpose();

  _jacobianOplus[6].setZero();
  _jacobianOplus[6].block<3, 3>(3, 0) = -preintegrator_->dv_dba();
  _jacobianOplus[6].block<3, 3>(6, 0) = -preintegrator_->dp_dba();

  _jacobianOplus[7].block<3, 3>(0, 0) = -Jr_inv * err_SO3.matrix().transpose() *
                                        preintegrator_->Jrb(bias_gi) *
                                        preintegrator_->dR_dbg();
  _jacobianOplus[7].block<3, 3>(3, 0) = -preintegrator_->dv_dbg();
  _jacobianOplus[7].block<3, 3>(6, 0) = -preintegrator_->dp_dbg();
}

void EdgePrior::computeError() {
  auto rot_i = static_cast<VertexSO3 *>(_vertices[0])->estimate();
  auto pos_i = static_cast<VertexPos *>(_vertices[1])->estimate();
  auto vel_i = static_cast<VertexVel *>(_vertices[2])->estimate();
  auto bias_ai = static_cast<VertexBiasA *>(_vertices[3])->estimate();
  auto bias_gi = static_cast<VertexBiasG *>(_vertices[4])->estimate();

  const Eigen::Vector3d err_rot = (prior_.rot.inverse() * rot_i).log();
  const Eigen::Vector3d err_vel = vel_i - prior_.vel;
  const Eigen::Vector3d err_pos = pos_i - prior_.pos;
  const Eigen::Vector3d err_ba = bias_ai - prior_.bias_a;
  const Eigen::Vector3d err_bg = bias_gi - prior_.bias_g;

  _error << err_rot, err_vel, err_pos, err_ba, err_bg;
}
void EdgePrior::linearizeOplus() {
  auto rot_i = static_cast<VertexSO3 *>(_vertices[0])->estimate();
  _jacobianOplus[0].setZero();
  _jacobianOplus[0].block<3, 3>(0, 0) =
      Sophus::SO3d::leftJacobianInverse(-(prior_.rot.inverse() * rot_i).log());

  _jacobianOplus[1].setZero();
  _jacobianOplus[1].block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();

  _jacobianOplus[2].setZero();
  _jacobianOplus[2].block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();

  _jacobianOplus[3].setZero();
  _jacobianOplus[3].block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();

  _jacobianOplus[4].setZero();
  _jacobianOplus[4].block<3, 3>(12, 0) = Eigen::Matrix3d::Identity();
}

void EdgeSE3::computeError() {
  Sophus::SO3d rot_i = static_cast<VertexSO3 *>(_vertices[0])->estimate();
  const Eigen::Vector3d pos_i =
      static_cast<VertexPos *>(_vertices[1])->estimate();
  const Eigen::Vector3d err_rot = (_measurement.so3().inverse() * rot_i).log();
  const Eigen::Vector3d err_pos = pos_i - _measurement.translation();
  _error << err_rot, err_pos;
}
void EdgeSE3::linearizeOplus() {
  Sophus::SO3d rot_i = static_cast<VertexSO3 *>(_vertices[0])->estimate();
  const Eigen::Vector3d err_rot = (_measurement.so3().inverse() * rot_i).log();

  _jacobianOplusXi.setZero();
  _jacobianOplusXi.block<3, 3>(0, 0) =
      Sophus::SO3d::leftJacobianInverse(-err_rot);
  _jacobianOplusXj.setZero();
  _jacobianOplusXj.block<3, 3>(3, 0).setIdentity();
}

void EdgeBiasA::computeError() {
  _error = static_cast<VertexBiasA *>(_vertices[1])->estimate() -
           static_cast<VertexBiasA *>(_vertices[0])->estimate();
}
void EdgeBiasA::linearizeOplus() {
  _jacobianOplusXi = -Eigen::Matrix3d::Identity();
  _jacobianOplusXj.setIdentity();
}

void EdgeBiasG::computeError() {
  _error = static_cast<VertexBiasA *>(_vertices[1])->estimate() -
           static_cast<VertexBiasA *>(_vertices[0])->estimate();
}
void EdgeBiasG::linearizeOplus() {
  _jacobianOplusXi = -Eigen::Matrix3d::Identity();
  _jacobianOplusXj.setIdentity();
}
