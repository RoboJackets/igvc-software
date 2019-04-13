#add_rostest_gtest(TestMapper test/test_mapper.test test_mapper.cpp)
#target_link_libraries(TestMapper ${catkin_LIBRARIES})

catkin_add_gtest(TestFastSweep test_fast_sweep.cpp)
target_link_libraries(TestFastSweep FastSweep)

catkin_add_gtest(TestSignedDistanceField test_signed_distance_field.cpp)
target_link_libraries(TestSignedDistanceField FastSweep ${catkin_LIBRARIES})

#add_rostest_gtest(TestPathPlanner test/test_path_planner.test test_path_planner.cpp)
#target_link_libraries(TestPathPlanner ${catkin_LIBRARIES})

#add_rostest_gtest(TestPathFollower test/test_path_follower.test test_path_follower.cpp)
#target_link_libraries(TestPathFollower ${catkin_LIBRARIES})
