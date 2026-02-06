#pragma once

#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sophus/se3.hpp>

class NDT_INC {
public:
  using VoxelId = Eigen::Vector3i;
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
  using Mat36 = Eigen::Matrix<double, 3, 6>;
  using Mat6 = Eigen::Matrix<double, 6, 6>;
  using Vec6 = Eigen::Matrix<double, 6, 1>;
  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix3d;
  using Mat18 = Eigen::Matrix<double, 18, 18>;
  using Mat3_18 = Eigen::Matrix<double, 3, 18>;
  using Vec18 = Eigen::Matrix<double, 18, 1>;

  struct Voxel {
    Voxel() = default;
    Voxel(const Vec3 &pos) : pts({pos}) {}

    std::vector<Vec3, Eigen::aligned_allocator<Vec3>> pts;

    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d info = Eigen::Matrix3d::Zero();

    size_t num_evaluated_pts = 0;
    bool initial_evaluated = false;
  };

  // Optimized Spatial Hashing for Collision Detection of Deformable Objects,
  // 2003
  struct hash_pt3 {
    size_t operator()(const Eigen::Vector3i &pt) const {
      return size_t((pt[0] * 73856093) ^ (pt[1] * 19349663) ^
                    (pt[2] * 83492791) % 10000000);
    }
  };

  enum class NeighborType { NB0, NB6, NB14, NB27 };

  struct Params {
    int iterations = 20;
    int min_valid = 10;
    double eps = 1e-3;
    NeighborType nb_type = NeighborType::NB6;
    double vx_size = 1.0; // unit: meter
    double max_dist = 1.0;
    double chi2_th = 20.0;
    bool guess_translation = false;
    size_t min_vx_pt = 5;
    size_t max_vx_pt = 50;
    size_t grid_capacity = 100000;
  };

  void set_params(const Params &param) {
    params_ = param;
    set_neighbors(params_.nb_type);
  }
  void set_neighbors(const NeighborType type);

  bool add_scan(PointCloudPtr scan);

  bool align(Sophus::SE3d &Tts, PointCloudPtr source);

  bool compute_Hb(const Sophus::SE3d &Tts, Mat18 &H, Vec18 &b);

  void set_source(PointCloudPtr source) { source_ = source; }

  size_t size() const { return grid_.size(); }

private:
  using VoxelInfo = std::pair<VoxelId, std::unique_ptr<Voxel>>;
  std::list<VoxelInfo> data_;
  std::unordered_map<VoxelId, std::list<VoxelInfo>::iterator, hash_pt3> grid_;
  PointCloudPtr source_;

  Params params_;

  bool initial_scan_inserted = false;

  std::vector<VoxelId, Eigen::aligned_allocator<VoxelId>> neighbors_;

  void evaluate_voxel(Voxel *voxel);

  bool mean_cov(const Voxel &voxel, Eigen::Vector3d &mean,
                Eigen::Matrix3d &cov) const;
  void update_mean_cov(Voxel &voxel, const Eigen::Vector3d &new_mean,
                       const Eigen::Matrix3d &new_cov);

  static Eigen::Matrix3d info_mat(const Eigen::Matrix3d &cov);

  VoxelId get_id(const Eigen::Vector3d &p) const {
    return (p / params_.vx_size).array().round().cast<int>();
  }

  const Eigen::Vector3d pos(const pcl::PointXYZI &pt) const {
    return pt.getVector3fMap().cast<double>();
  }
};