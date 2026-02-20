#include "lio/LioPreint.h"
#include "g2o_types/types.h"

#include <execution>

bool LioPreint::config(const std::string &yaml_file) {

  lidar_imu_sync_ = Sync([this](const Sync::DataGroup &lidar_imu) {
    process_sync_data(lidar_imu);
  });
  lidar_imu_sync_.config(yaml_file);

  imu_initializer_.config(yaml_file);

  if (params_.viewer_on) {
    viewer_ = std::make_unique<MapViewer>("IESKF LIO", 0.5);
  }

  NDT_INC::Params ndt_params;
  ndt_params.nb_type = NDT_INC::NeighborType::NB6;
  //   ndt_params.nb_type = NDT_INC::NeighborType::NB0;
  ndt_params.vx_size = 1.0;
  ndt_params.min_vx_pt = 5;
  ndt_params.iterations = 5;
  ndt_params.chi2_th = 5.0;
  ndt_params.guess_translation = false;
  ndt_inc_.set_params(ndt_params);

  auto yaml = YAML::LoadFile(yaml_file);
  std::vector<double> ext_t =
      yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
  std::vector<double> ext_r =
      yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();

  const Eigen::Vector3d lidar_T_wrt_IMU = VecFromArray(ext_t);
  const Eigen::Matrix3d lidar_R_wrt_IMU = MatFromArray(ext_r);
  T_IL_ = Sophus::SE3d(lidar_R_wrt_IMU, lidar_T_wrt_IMU);

  build_info();

  return true;
}

void LioPreint::build_info() {
  double coeff = 1.0 / (params_.sigma_bg * params_.sigma_bg);
  bg_info_.diagonal() << coeff, coeff, coeff;

  coeff = 1.0 / (params_.sigma_ba * params_.sigma_ba);
  ba_info_.diagonal() << coeff, coeff, coeff;

  coeff = 1.0 / (params_.sigma_ndt_ang * params_.sigma_ndt_ang);
  const double coeff_pos =
      1.0 / (params_.sigma_ndt_pos * params_.sigma_ndt_pos);
  ndt_info_.diagonal() << coeff, coeff, coeff, coeff_pos, coeff_pos, coeff_pos;
}

void LioPreint::add_imu(std::unique_ptr<sensor_msgs::msg::Imu> imu_msg) {
  lidar_imu_sync_.add_imu(std::move(imu_msg));
}

void LioPreint::add_scan(
    std::unique_ptr<sensor_msgs::msg::PointCloud2> scan_msg) {
  lidar_imu_sync_.add_cloud(std::move(scan_msg));
}

void LioPreint::process_sync_data(const Sync::DataGroup &lidar_imu) {
  lidar_imu_ = lidar_imu;
  if (!imu_initializer_.success()) {
    initialize_imu(lidar_imu_);
    return;
  }

  predict();

  undistort();

  correct();
}

bool LioPreint::initialize_imu(const Sync::DataGroup &lidar_imu) {
  for (const auto &imu : lidar_imu.imu_sequence) {
    if (imu_initializer_.addImu(*imu)) {
      break;
    }
  }
  if (imu_initializer_.success()) {
    last_state_ = IMUState();
    last_state_.pos = Eigen::Vector3d::Zero();
    last_state_.vel = Eigen::Vector3d::Zero();
    last_state_.rot = Sophus::SO3d();
    last_state_.bias_g = imu_initializer_.bias_g();
    last_state_.bias_a = imu_initializer_.bias_a();
    last_state_.gravity = imu_initializer_.gravity();
    last_state_.timestamp = imu_initializer_.timestamp();
    last_kf_pose_ = Sophus::SE3d();

    // const Eigen::Vector3d var_g =
    //     Eigen::Vector3d::Ones() * imu_initializer_.var_g()[0];
    // const Eigen::Vector3d var_a =
    //     Eigen::Vector3d::Ones() * imu_initializer_.var_a()[0];

    preint_ =
        IMUPreintegrator(imu_initializer_.bias_g(), imu_initializer_.bias_a(),
                         imu_initializer_.var_g(), imu_initializer_.var_a(),
                         imu_initializer_.timestamp());

    preint_.set_gravity(imu_initializer_.gravity());

    preint_initialized_ = true;
  }
  return preint_initialized_;
}

void LioPreint::predict() {
  states_.clear();
  states_.reserve(lidar_imu_.imu_sequence.size() + 1);
  states_.emplace_back(last_state_);

  for (const auto &imu_data : lidar_imu_.imu_sequence) {
    preint_.integrate(*imu_data);
    states_.emplace_back(preint_.predict(last_state_));
  }

  return;
}

// p_li to p_le
void LioPreint::undistort() {
  auto &pts = lidar_imu_.scan->points;
  std::sort(pts.begin(), pts.end(),
            [](const LidarPointType &pt_a, const LidarPointType &pt_b) {
              return pt_a.time < pt_b.time;
            });

  const double last_imu_time = states_.back().timestamp;
  const Sophus::SE3d T_W_Ie(states_.back().rot, states_.back().pos);

  size_t sid_end = 1;

  for (auto &pt : pts) {
    const double ptime = lidar_imu_.scan_start_time + pt.time * 1e-3;
    Sophus::SE3d T_W_Ii;

    if (ptime > last_imu_time) {
      T_W_Ii = integrate_imu(states_.back(), lidar_imu_.imu_sequence.back(),
                             ptime - last_imu_time);
      // T_W_Ii = T_W_Ie;

    } else {

      while (states_[sid_end].timestamp < ptime) {
        sid_end++;
      }

      const double dt =
          states_[sid_end].timestamp - states_[sid_end - 1].timestamp;
      if (dt < 1e-6) {
        LOG(WARNING) << "time window is too small: " << dt;
        T_W_Ii = Sophus::SE3d(states_[sid_end].rot, states_[sid_end].pos);

      } else {
        const double ratio = (ptime - states_[sid_end - 1].timestamp) / dt;
        T_W_Ii = interpolation(ratio, states_[sid_end - 1], states_[sid_end]);
      }
    }

    const Eigen::Vector3d p_Ie =
        T_W_Ie.inverse() * T_W_Ii * T_IL_ * pt.getVector3fMap().cast<double>();
    pt.x = float(p_Ie.x());
    pt.y = float(p_Ie.y());
    pt.z = float(p_Ie.z());
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
LioPreint::desampling(const double leaf_sz) {

  LidarPointCloudPtr cloud_I = lidar_imu_.scan;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl_cloud->points.resize(cloud_I->size());
  std::vector<size_t> idx(cloud_I->size());
  std::iota(idx.begin(), idx.end(), 0);
  std::for_each(std::execution::par_unseq, idx.begin(), idx.end(),
                [&pcl_cloud, &cloud_I](const size_t pid) {
                  pcl_cloud->points[pid].x = cloud_I->points[pid].x;
                  pcl_cloud->points[pid].y = cloud_I->points[pid].y;
                  pcl_cloud->points[pid].z = cloud_I->points[pid].z;
                  pcl_cloud->points[pid].intensity = 0;
                });
  pcl_cloud->height = 1;
  pcl_cloud->width = pcl_cloud->size();

  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setLeafSize(leaf_sz, leaf_sz, leaf_sz);
  vg.setInputCloud(pcl_cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr desmapled_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  vg.filter(*desmapled_cloud);

  return desmapled_cloud;
}

void LioPreint::correct() {

  auto pcl_cloud = desampling(0.5);

  if (!ndt_initialized) {
    ndt_inc_.add_scan(pcl_cloud);

    if (viewer_) {
      viewer_->add_pointcloud(pcl_cloud, Sophus::SE3d());
    }
    ndt_initialized = true;
    return;
  }

  cur_state_ = preint_.predict(last_state_);
  aligned_pose_ = cur_state_.SE3();

  LOG(INFO) << "before correction: " << aligned_pose_.translation().transpose();
  ndt_inc_.align(aligned_pose_, pcl_cloud);
  LOG(INFO) << "after correction: " << aligned_pose_.translation().transpose();

  optimize();

  const Sophus::SE3d cur_pose = cur_state_.SE3();
  if (is_keyframe(cur_pose)) {
    auto cloud_W = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::transformPointCloud(*pcl_cloud, *cloud_W,
                             cur_pose.matrix().cast<float>());
    LOG(INFO) << "Creating new KeyFrame at "
              << cur_pose.translation().transpose();
    ndt_inc_.add_scan(cloud_W);

    if (viewer_) {
      viewer_->add_pointcloud(cloud_W, cur_pose);
    }
    last_kf_pose_ = cur_pose;
  }

  //   const Eigen::Vector3d var_g =
  //       Eigen::Vector3d::Ones() * imu_initializer_.var_g()[0];
  //   const Eigen::Vector3d var_a =
  //       Eigen::Vector3d::Ones() * imu_initializer_.var_a()[0];

  preint_.reset(cur_state_.bias_g, cur_state_.bias_a, imu_initializer_.var_g(),
                imu_initializer_.var_a(), cur_state_.timestamp);
  last_state_ = cur_state_;
}

bool LioPreint::is_keyframe(const Sophus::SE3d &pose) const {
  const Sophus::SE3d rel_motion = last_kf_pose_.inverse() * pose;
  return (rel_motion.translation().norm() > params_.min_kf_dist) ||
         (rel_motion.so3().log().norm() > params_.min_kf_deg * deg2rad);
}

Sophus::SE3d LioPreint::integrate_imu(const IMUState &state,
                                      const IMUPtr &imu_measure,
                                      const double dt) const {
  const Eigen::Vector3d last_acc = imu_measure->acc;
  const Eigen::Vector3d last_omega = imu_measure->gyr;

  const Eigen::Vector3d pos =
      state.pos + state.vel * dt + 0.5 * state.gravity * dt * dt +
      0.5 * (state.rot * (last_acc - state.bias_a)) * dt * dt;
  const Sophus::SO3d rot =
      state.rot * Sophus::SO3d::exp((last_omega - state.bias_g) * dt);
  return Sophus::SE3d(rot, pos);
}

Sophus::SE3d LioPreint::interpolation(const double ratio,
                                      const IMUState &state0,
                                      const IMUState &state1) const {
  const Sophus::SE3d pose0(state0.rot, state0.pos);
  const Sophus::SE3d pose1(state1.rot, state1.pos);

  const Eigen::Vector3d pos =
      (1 - ratio) * pose0.translation() + ratio * pose1.translation();
  const Sophus::SO3d rot = Sophus::SO3d(
      pose0.unit_quaternion().slerp(ratio, pose1.unit_quaternion()));

  return Sophus::SE3d(rot, pos);
}

void LioPreint::optimize() {
  using BlockSolverType = g2o::BlockSolverX;
  using LinearSolverType =
      g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

  auto *solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  size_t vid = 0;
  auto rot_i = new VertexSO3();
  rot_i->setId(vid++);
  rot_i->setEstimate(last_state_.rot);
  optimizer.addVertex(rot_i);

  auto pos_i = new VertexPos();
  pos_i->setId(vid++);
  pos_i->setEstimate(last_state_.pos);
  optimizer.addVertex(pos_i);

  auto vel_i = new VertexVel();
  vel_i->setId(vid++);
  vel_i->setEstimate(last_state_.vel);
  optimizer.addVertex(vel_i);

  auto bg_i = new VertexBiasG();
  bg_i->setId(vid++);
  bg_i->setEstimate(last_state_.bias_g);
  optimizer.addVertex(bg_i);

  auto ba_i = new VertexBiasA();
  ba_i->setId(vid++);
  ba_i->setEstimate(last_state_.bias_a);
  optimizer.addVertex(ba_i);

  auto rot_j = new VertexSO3();
  rot_j->setId(vid++);
  //   rot_j->setEstimate(cur_state_.rot);
  rot_j->setEstimate(aligned_pose_.so3());
  optimizer.addVertex(rot_j);

  auto pos_j = new VertexPos();
  pos_j->setId(vid++);
  //   pos_j->setEstimate(cur_state_.pos);
  pos_j->setEstimate(aligned_pose_.translation());
  optimizer.addVertex(pos_j);

  auto vel_j = new VertexVel();
  vel_j->setId(vid++);
  vel_j->setEstimate(cur_state_.vel);
  optimizer.addVertex(vel_j);

  auto bg_j = new VertexBiasG();
  bg_j->setId(vid++);
  bg_j->setEstimate(cur_state_.bias_g);
  optimizer.addVertex(bg_j);

  auto ba_j = new VertexBiasA();
  ba_j->setId(vid++);
  ba_j->setEstimate(cur_state_.bias_a);
  optimizer.addVertex(ba_j);

  size_t eid = 0;
  auto edge_preint =
      new EdgePreint(&preint_, cur_state_.timestamp - last_state_.timestamp);
  edge_preint->setId(eid++);
  edge_preint->setVertex(0, rot_i);
  edge_preint->setVertex(1, rot_j);
  edge_preint->setVertex(2, vel_i);
  edge_preint->setVertex(3, vel_j);
  edge_preint->setVertex(4, pos_i);
  edge_preint->setVertex(5, pos_j);
  edge_preint->setVertex(6, ba_i);
  edge_preint->setVertex(7, bg_i);
  auto *rk_huber = new g2o::RobustKernelHuber();
  rk_huber->setDelta(200.0);
  edge_preint->setRobustKernel(rk_huber);
  edge_preint->setInformation(preint_.info());
  optimizer.addEdge(edge_preint);

  auto edge_prior = new EdgePrior(last_state_);
  edge_prior->setId(eid++);
  edge_prior->setVertex(0, rot_i);
  edge_prior->setVertex(1, pos_i);
  edge_prior->setVertex(2, vel_i);
  edge_prior->setVertex(3, bg_i);
  edge_prior->setVertex(4, ba_i);
  edge_prior->setInformation(prior_info_);
  optimizer.addEdge(edge_prior);

  auto edge_ba = new EdgeBiasA();
  edge_ba->setId(eid++);
  edge_ba->setVertex(0, ba_i);
  edge_ba->setVertex(1, ba_j);
  edge_ba->setInformation(ba_info_);
  optimizer.addEdge(edge_ba);

  auto edge_bg = new EdgeBiasG();
  edge_bg->setId(eid++);
  edge_bg->setVertex(0, bg_i);
  edge_bg->setVertex(1, bg_j);
  edge_bg->setInformation(bg_info_);
  optimizer.addEdge(edge_bg);

  auto edge_ndt = new EdgeSE3();
  edge_ndt->setId(eid++);
  edge_ndt->setVertex(0, rot_j);
  edge_ndt->setVertex(1, pos_j);
  edge_ndt->setMeasurement(aligned_pose_);
  edge_ndt->setInformation(ndt_info_);
  optimizer.addEdge(edge_ndt);

  ba_i->setFixed(true);
  bg_i->setFixed(true);

  optimizer.initializeOptimization();
  optimizer.optimize(params_.iterations);

  cur_state_.rot = rot_j->estimate();
  cur_state_.pos = pos_j->estimate();
  cur_state_.vel = vel_j->estimate();
  cur_state_.bias_a = ba_j->estimate();
  cur_state_.bias_g = bg_j->estimate();

  // rot_i, pos_i, vel_i, bg_i, ba_i, rot_j, pos_j, vel_j, bg_j, ba_j
  //     0,     3,     6,    9,   12,    15,    18,    21,   24,   27
  Eigen::Matrix<double, 30, 30> H = Eigen::Matrix<double, 30, 30>::Zero();

  // rot_i, pos_i, vel_i, bg_i, ba_i
  H.block<15, 15>(0, 0) += edge_prior->Hessian();

  // rot_j, pos_j
  H.block<6, 6>(15, 15) += edge_ndt->Hessian();

  // bg_i, bg_j
  //    9,   24
  const Eigen::Matrix<double, 6, 6> &H_bg = edge_bg->Hessian();
  H.block<3, 3>(9, 9) += H_bg.block<3, 3>(0, 0);
  H.block<3, 3>(9, 24) += H_bg.block<3, 3>(0, 3);
  H.block<3, 3>(24, 9) += H_bg.block<3, 3>(3, 0);
  H.block<3, 3>(24, 24) += H_bg.block<3, 3>(3, 3);

  // ba_i, ba_j
  //   12,   27
  const Eigen::Matrix<double, 6, 6> &H_ba = edge_ba->Hessian();
  H.block<3, 3>(12, 12) += H_ba.block<3, 3>(0, 0);
  H.block<3, 3>(12, 27) += H_ba.block<3, 3>(0, 3);
  H.block<3, 3>(27, 12) += H_ba.block<3, 3>(3, 0);
  H.block<3, 3>(27, 27) += H_ba.block<3, 3>(3, 3);

  // rot_i, rot_j, vel_i, vel_j, pos_i, pos_j, ba_i, bg_i
  //     0,    15,     6,    21,     3,    18,   12,    9
  const Eigen::Matrix<double, 24, 24> H_preint = edge_preint->Hessian();
  std::vector<int> idx = {0, 15, 6, 21, 3, 18, 12, 9};
  for (size_t i = 0; i < idx.size(); ++i) {
    for (size_t j = 0; j < idx.size(); ++j) {
      H.block<3, 3>(idx[i], idx[j]) += H_preint.block<3, 3>(3 * i, 3 * j);
    }
  }

  marginalize(H);

  LOG(INFO) << "prior trace:" << prior_info_.trace();
  // normalize_vel();
}

void LioPreint::marginalize(const Eigen::Matrix<double, 30, 30> &H) {
  prior_info_.setZero();

  const Eigen::Matrix<double, 15, 15> &A = H.block<15, 15>(0, 0);
  const Eigen::Matrix<double, 15, 15> &B = H.block<15, 15>(0, 15);
  const Eigen::Matrix<double, 15, 15> &Bt = H.block<15, 15>(15, 0);
  const Eigen::Matrix<double, 15, 15> &C = H.block<15, 15>(15, 15);

  Eigen::JacobiSVD svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix<double, 15, 1> singulars = svd.singularValues();
  for (int i = 0; i < 15; ++i) {
    if (singulars[i] < 1e-6) {
      singulars[i] = 0;
    } else {
      singulars[i] = 1.0 / singulars[i];
    }
  }
  const Eigen::Matrix<double, 15, 15> A_inv =
      svd.matrixV() * singulars.asDiagonal() * svd.matrixU().transpose();

  prior_info_ = C - Bt * A_inv * B;
}

void LioPreint::normalize_vel() {
  Eigen::Vector3d vel_body = cur_state_.rot.inverse() * cur_state_.vel;

  // from -0.1 to 0.1
  vel_body[0] = std::min(vel_body[0], 0.1);
  vel_body[0] = std::max(vel_body[0], -0.1);

  // from -2.0  to 0.0
  vel_body[1] = std::min(vel_body[1], 0.0);
  vel_body[1] = std::max(vel_body[1], -2.0);

  vel_body[2] = 0;

  cur_state_.vel = cur_state_.rot * vel_body;
}

void LioPreint::save_map(const std::string &map_file) {
  if (viewer_) {
    viewer_->save_map(map_file);
    LOG(INFO) << "Saved map to " << map_file;
    LOG(INFO) << "Close viewer to stop the program.";
    viewer_->spin();
  }
}
