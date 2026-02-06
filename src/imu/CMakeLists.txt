add_library(imu imu.cpp ImuInitializer.cpp)
target_link_libraries(imu Eigen3::Eigen3 Sophus::Sophus glog::glog)
target_include_directories(imu PUBLIC ${CMAKE_SOURCE_DIR}/include/lio_preint)