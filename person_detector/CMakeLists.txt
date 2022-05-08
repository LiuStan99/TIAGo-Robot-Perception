cmake_minimum_required(VERSION 2.8.3)
project(person_detector)

find_package(catkin REQUIRED COMPONENTS image_transport
                                        pal_detection_msgs
                                        cv_bridge
                                        roscpp
  					vision_msgs
  					pcl_ros
  					pcl_conversions
  					sensor_msgs)
find_package(OpenCV REQUIRED)


catkin_package(
  CATKIN_DEPENDS image_transport cv_bridge pal_detection_msgs roscpp
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(person_detector src/person_detector.cpp)
target_link_libraries(person_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS person_detector
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
