include_directories (${selfr_SOURCE_DIR}/filters)
link_directories (${selfr_BINARY_DIR}/filters)

add_executable (SimpleKalmanTest SimpleKalmanTest.cc)
target_link_libraries(SimpleKalmanTest Filters flens fftw3)
