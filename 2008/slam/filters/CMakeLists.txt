#include_directories (${slam_SOURCE_DIR}/models)
#link_directories (${slam_BINARY_DIR}/models)

add_library(Filters KalmanFilter.cc)

target_link_libraries(Filters flens)
