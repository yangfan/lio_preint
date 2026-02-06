#define PCL_NO_PRECOMPILE
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct EIGEN_ALIGN16 LidarPointType {
  PCL_ADD_POINT4D;
  double time = 0.0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    LidarPointType, (float, x, x)(float, y, y)(float, z, z)(double, time, time))
using LidarPointCloud = pcl::PointCloud<LidarPointType>;
using LidarPointCloudPtr = pcl::PointCloud<LidarPointType>::Ptr;

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
    velodyne_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        float, time, time)(std::uint16_t, ring, ring))
