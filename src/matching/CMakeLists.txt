add_library(NDT_INC NDT_INC.cpp)
target_link_libraries(NDT_INC PUBLIC Eigen3::Eigen Sophus::Sophus ${PCL_LIBRARIES} TBB::tbb)
target_include_directories(NDT_INC PUBLIC ${CMAKE_SOURCE_DIR}/include/lio_preint/matching)