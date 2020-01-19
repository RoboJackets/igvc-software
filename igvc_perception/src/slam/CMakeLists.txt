# Find and add Boost
find_package(Boost REQUIRED)
find_package(Boost COMPONENTS system filesystem timer REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
list(APPEND NEEDED_LIBRARIES ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
if(GTSAM_FOUND)
	get_target_property(GTSAM_INCLUDE_DIR gtsam INTERFACE_INCLUDE_DIRECTORIES)
	set(GTSAM_LIBRARIES gtsam)
	include_directories(${GTSAM_INCLUDE_DIR})
	message(STATUS "Found gtsam, compiling slam")
	add_executable(slam
		main.cpp
		slam.cpp
		type_conversions.cpp)
	target_include_directories(slam PRIVATE ${catkin_INCLUDE_DIRS})
	target_link_libraries(slam gtsam ${catkin_LIBRARIES} Eigen3::Eigen ${Boost_LIBRARIES})
	add_dependencies(slam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
else()
	message(WARNING "Could not find gtsam. Not compiling slam!")
endif()
