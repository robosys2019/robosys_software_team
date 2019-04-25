CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

project(robosys)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

# For OpenCV
find_package( OpenCV REQUIRED )
# For libfreenect2
find_package( freenect2 REQUIRED HINTS "$ENV{HOME}/freenect2")

catkin_package()

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${freenect2_INCLUDE_DIRS}
)

add_executable(KinectOneStream
  src/KinectOneStream.cpp
)

target_link_libraries(KinectOneStream
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${freenect2_LIBRARIES}
)

