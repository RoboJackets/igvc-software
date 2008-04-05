project(GPSdriver)

add_library(GPS gps.cc thread.cc)

target_link_libraries(GPS pthread.so)
