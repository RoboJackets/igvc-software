add_rostest_gtest(TestJoystickDriver test/test_joystick_driver.test test_joystick_driver.cpp)
target_link_libraries(TestJoystickDriver ${catkin_LIBRARIES})

add_rostest_gtest(TestNewMapper test/test_new_mapper.test test_new_mapper.cpp)
target_link_libraries(TestNewMapper ${catkin_LIBRARIES})

add_rostest_gtest(TestFilterLidar test/test_filter_lidar.test test_filter_lidar.cpp)
target_link_libraries(TestNewMapper ${catkin_LIBRARIES})

#add_rostest_gtest(TestPathPlanner test/test_path_planner.test test_path_planner.cpp)
#target_link_libraries(TestPathPlanner ${catkin_LIBRARIES})

#add_rostest_gtest(TestPathFollower test/test_path_follower.test test_path_follower.cpp)
#target_link_libraries(TestPathFollower ${catkin_LIBRARIES})
