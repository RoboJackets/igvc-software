include_directories (${SLAM_SOURCE_DIR}/filters)
link_directories (${SLAM_BINARY_DIR}/filters)

include_directories (${SLAM_SOURCE_DIR}/pdf)
link_directories (${SLAM_BINARY_DIR}/pdf)

include_directories (${SLAM_SOURCE_DIR}/plot)
link_directories (${SLAM_BINARY_DIR}/plot)

add_executable (SimpleKalmanTest SimpleKalmanTest.cc)
target_link_libraries(SimpleKalmanTest Filters flens)

add_executable (Test Test.cc)
target_link_libraries (Test flens gsl fftw3)
