add_library(g2o_types types.cpp)
target_include_directories(g2o_types PUBLIC
    ${G2O_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include/lio_preint/g2o_types
    ${CMAKE_SOURCE_DIR}/include/lio_preint/
)

set(G2O_LIBS fmt ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION})

target_link_libraries(g2o_types
    ${G2O_LIBS}
    Eigen3::Eigen
    Sophus::Sophus
)