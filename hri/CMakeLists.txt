cmake_minimum_required(VERSION 3.0.2)
project(hri)


find_package(catkin REQUIRED COMPONENTS 
                            actionlib 
                            control_msgs 
                            controller_manager_msgs
                            play_motion_msgs 
                            moveit_ros_planning_interface 
                            roscpp 
                            sensor_msgs 
                            std_msgs
                            diagnostic_msgs 
                            diagnostic_updater)


find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
     INCLUDE_DIRS include
     CATKIN_DEPENDS actionlib 
                    control_msgs 
                    controller_manager_msgs
                    play_motion_msgs 
                    moveit_ros_planning_interface 
                    roscpp 
                    sensor_msgs 
                    std_msgs
                    diagnostic_msgs 
                    diagnostic_updater
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hri src/hri_wave_node.cpp src/wave.cpp)

add_dependencies(hri ${catkin_EXPORTED_TARGETS})
target_link_libraries(hri ${catkin_LIBRARIES})
