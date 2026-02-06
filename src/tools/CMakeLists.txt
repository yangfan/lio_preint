add_library(tools MapViewer.cpp BagIO.cpp PointCloudConverter.cpp Sync.cpp)
target_link_libraries(tools ${PCL_LIBRARIES} yaml-cpp::yaml-cpp Eigen3::Eigen Sophus::Sophus rclcpp::rclcpp rosbag2_transport::rosbag2_transport pcl_conversions::pcl_conversions ${sensor_msgs_TARGETS})
target_include_directories(tools PUBLIC
    ${CMAKE_SOURCE_DIR}/include/lio_preint/tools
    ${CMAKE_SOURCE_DIR}/include/lio_preint/
)