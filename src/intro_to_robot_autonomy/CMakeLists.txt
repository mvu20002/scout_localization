cmake_minimum_required(VERSION 2.8.3)
project(intro_to_robot_autonomy)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS    
    roslaunch 
    roslint 
    roscpp 
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2 
    tf2_ros
    nav_msgs
    tf
    velocity_controllers
    robot_localization
    rqt_multiplot
    hector_gazebo_plugins
    cv_bridge
    tf
    image_transport
    roslint
    xacro
    robot_state_publisher
    gazebo_plugins
    joint_state_publisher
    gazebo_ros_control
)
catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES 
  CATKIN_DEPENDS roscpp sensor_msgs
#   DEPENDS Boost
)

find_package(YAML-CPP REQUIRED)


###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)


add_executable(circle_mov_command src/circle_mov_command.cpp)
target_link_libraries(circle_mov_command ${catkin_LIBRARIES})


add_executable(publish_odom_from_tf src/publish_odom_from_tf.cpp)
target_link_libraries(publish_odom_from_tf ${catkin_LIBRARIES})

add_executable(pub_odom_from_robot_state src/pub_odom_from_robot_state.cpp)
target_link_libraries(pub_odom_from_robot_state ${catkin_LIBRARIES})

add_executable(gps_accuracy_checker src/gps_acc.cpp)
target_link_libraries(gps_accuracy_checker ${catkin_LIBRARIES})

add_executable(imu_accuracy_checker src/imu_acc.cpp)
target_link_libraries(imu_accuracy_checker ${catkin_LIBRARIES})

add_executable(parametric_velocity_sequential src/vel_commands.cpp)
target_link_libraries(parametric_velocity_sequential ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS src/live_map.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(parametric_velocity_sequential ${YAML_CPP_LIBRARIES})
                      

#############
## Install ##
#############

roslaunch_add_file_check(launch)

