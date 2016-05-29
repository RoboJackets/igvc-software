find_package(PkgConfig REQUIRED)
pkg_search_module(GLIBTOP REQUIRED libgtop-2.0)


add_executable(system_stats main.cpp)
target_link_libraries(system_stats ${catkin_LIBRARIES} ${GLIBTOP_LIBRARIES})
include_directories(${GLIBTOP_INCLUDE_DIRS})
target_compile_options(system_stats PUBLIC ${GLIBTOP_CFLAGS_OTHER})
