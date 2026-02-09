add_library(lio_preint LioPreint.cpp)
target_link_libraries(lio_preint imu tools g2o_types NDT_INC yaml-cpp::yaml-cpp Eigen3::Eigen Sophus::Sophus ${PCL_LIBRARIES} rclcpp::rclcpp ${sensor_msgs_TARGETS})
target_include_directories(lio_preint PRIVATE ${CMAKE_SOURCE_DIR}/include/lio_preint)