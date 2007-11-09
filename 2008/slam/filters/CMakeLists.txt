#include_directories (${SLAM_SOURCE_DIR}/models)
#link_directories (${SLAM_BINARY_DIR}/models)

add_library(Filters KalmanFilter.cc)

target_link_libraries(Filters flens)
