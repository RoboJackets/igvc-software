cmake_minimum_required(VERSION 2.8.3)
project(igvc_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
             message_generation
             std_msgs
             sensor_msgs
             geometry_msgs
             actionlib_msgs
             )

# Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES
    velocity_pair.msg
    velocity_quad.msg
    lights.msg
    action_path.msg
    system_stats.msg
    map.msg
    igvc_path.msg
    trajectory_point.msg
    trajectory.msg
    barrels.msg
)

add_service_files(
    DIRECTORY srv
    FILES
    BackCircle.srv
)

add_action_files(
    DIRECTORY action
    FILES
    NavigateWaypoint.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs actionlib_msgs message_runtime
)



