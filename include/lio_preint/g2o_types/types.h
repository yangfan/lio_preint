#pragma once
#include <Eigen/Core>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <iostream>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include "imu/imu.h"

class VertexVec3 : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  VertexVec3() {}
  virtual bool read(std::istream &is) override {
    is.clear();
    return false;
  }
  virtual bool write(std::ostream &os) const override {
    os.clear();
    return false;
  }
  virtual void setToOriginImpl() override { _estimate.setZero(); }
  virtual void oplusImpl(const double *update) override;
};

class VertexSO3 : public g2o::BaseVertex<3, Sophus::SO3d> {
  virtual bool read(std::istream &is) override {
    is.clear();
    return false;
  }
  virtual bool write(std::ostream &os) const override {
    os.clear();
    return false;
  }
  virtual void setToOriginImpl() override { _estimate = Sophus::SO3d(); }
  virtual void oplusImpl(const double *update) override {
    _estimate *= Sophus::SO3d::exp(Eigen::Map<const Eigen::Vector3d>(update));
  };
};

class VertexPos : public VertexVec3 {
public:
  VertexPos() {}
};

class VertexVel : public VertexVec3 {
public:
  VertexVel() {}
};

class VertexBiasG : public VertexVec3 {
public:
  VertexBiasG() {}
};

class VertexBiasA : public VertexVec3 {
public:
  VertexBiasA() {}
};

class EdgePreint : public g2o::BaseMultiEdge<9, Eigen::Matrix<double, 9, 1>> {
public:
  EdgePreint(IMUPreintegrator *preint, const double t_ij)
      : preintegrator_(preint), t_ij_(t_ij) {
    resize(8);
  }
  virtual bool read(std::istream &is) override {
    is.clear();
    return false;
  }
  virtual bool write(std::ostream &os) const override {
    os.clear();
    return false;
  }

  virtual void computeError() override;
  virtual void linearizeOplus() override;

  Eigen::Matrix<double, 24, 24> Hessian();

private:
  IMUPreintegrator *preintegrator_;
  double t_ij_ = 0.0;
};

class EdgePrior : public g2o::BaseMultiEdge<15, Eigen::Matrix<double, 15, 1>> {
public:
  EdgePrior(const IMUState &prior) : prior_(prior) { resize(5); }
  virtual bool read(std::istream &is) override {
    is.clear();
    return false;
  }
  virtual bool write(std::ostream &os) const override {
    os.clear();
    return false;
  }

  virtual void computeError() override;
  virtual void linearizeOplus() override;

  Eigen::Matrix<double, 15, 15> Hessian();

private:
  IMUState prior_;
};

class EdgeSE3
    : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexSO3, VertexPos> {
public:
  virtual bool read(std::istream &is) override {
    is.clear();
    return false;
  }
  virtual bool write(std::ostream &os) const override {
    os.clear();
    return false;
  }

  virtual void computeError() override;
  virtual void linearizeOplus() override;

  Eigen::Matrix<double, 6, 6> Hessian();
};

class EdgeBiasA
    : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexBiasA, VertexBiasA> {
public:
  virtual bool read(std::istream &is) override {
    is.clear();
    return false;
  }
  virtual bool write(std::ostream &os) const override {
    os.clear();
    return false;
  }

  virtual void computeError() override;
  virtual void linearizeOplus() override;

  Eigen::Matrix<double, 6, 6> Hessian();
};

class EdgeBiasG
    : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexBiasG, VertexBiasG> {
public:
  virtual bool read(std::istream &is) override {
    is.clear();
    return false;
  }
  virtual bool write(std::ostream &os) const override {
    os.clear();
    return false;
  }

  virtual void computeError() override;
  virtual void linearizeOplus() override;

  Eigen::Matrix<double, 6, 6> Hessian();
};
