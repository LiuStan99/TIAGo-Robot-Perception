cmake_minimum_required(VERSION 3.0.2)
project(hri)


find_package(catkin REQUIRED COMPONENTS 
                            actionlib_msgs
                            genmsg
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

add_action_files(DIRECTORY action FILES speech.action gesture.action)
generate_messages(DEPENDENCIES actionlib_msgs)


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

add_executable(hri src/hri_gesture_node.cpp src/make_gesture.cpp)

add_dependencies(hri ${catkin_EXPORTED_TARGETS})
target_link_libraries(hri ${catkin_LIBRARIES})
