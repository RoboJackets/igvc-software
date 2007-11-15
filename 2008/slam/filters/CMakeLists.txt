include_directories (${SLAM_SOURCE_DIR}/pdf)
link_directories (${SLAM_BINARY_DIR}/pdf)

add_library (Filters KalmanFilter.cc)
target_link_libraries (Filters flens)
