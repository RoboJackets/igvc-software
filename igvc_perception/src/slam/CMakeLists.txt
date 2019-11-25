# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)

if (GTSAMCMakeTools_FOUND)
    include(GtsamBuildTypes) # Load build type flags and default to Debug mode
    include(GtsamTesting)    # Easy functions for creating unit tests and scripts

    # Find GTSAM components
    find_package(GTSAM REQUIRED) # Uses installed package
    include_directories(${GTSAM_INCLUDE_DIR})
    if(GTSAM_FOUND)
        message(STATUS "Found gtsam, compiling slam")
        find_package(Eigen3 3.3 REQUIRED)
        add_executable(slam
                main.cpp
                slam.cpp)
        target_include_directories(slam PRIVATE ${catkin_INCLUDE_DIRS})
        target_link_libraries(slam gtsam ${catkin_LIBRARIES} Eigen3::Eigen)
        add_dependencies(slam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    else()
        message(WARNING "Could not find gtsam, not compiling slam!")
    endif()
else()
    message(WARNING "Could not find GTSAMCMakeTools. Not compiling SLAM")
endif ()