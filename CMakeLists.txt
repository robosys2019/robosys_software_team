CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# For OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

INCLUDE_DIRECTORIES(
  ${freenect2_INCLUDE_DIR}
)

ADD_EXECUTABLE(KinectOneStream
  src/KinectOneStream.cpp
)

TARGET_LINK_LIBRARIES(KinectOneStream
  ${OpenCV_LIBS}
  ${freenect2_LIBRARIES}
)
