include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG main
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(preint_test preint_test.cpp)

target_link_libraries(preint_test lio_preint tools gtest glog::glog ${GFLAGS_LIBRARIES} Eigen3::Eigen Sophus::Sophus)
target_include_directories(preint_test PRIVATE
    ${CMAKE_SOURCE_DIR}/include/lio_preint
)

include(GoogleTest)
gtest_discover_tests(preint_test)