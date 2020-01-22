cmake_minimum_required(VERSION 2.8.3)
project(igvc_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
             message_generation
             std_msgs
             sensor_msgs
             geometry_msgs
             nav_msgs
             )

# Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES
    velocity_pair.msg
    lights.msg
    action_path.msg
    system_stats.msg
    map.msg
    igvc_path.msg
    trajectory_point.msg
    trajectory.msg
)

# Generate services in the "srv" folder
add_service_files(
    DIRECTORY srv
    FILES
    fake_cone.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs nav_msgs message_runtime
)

