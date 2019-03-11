CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# For OpenCV
find_package( OpenCV REQUIRED )
# For libfreenect2
find_package( freenect2 REQUIRED HINTS "$ENV{HOME}/freenect2")

include_directories( include
  ${OpenCV_INCLUDE_DIRS}
  ${freenect2_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(
)

ADD_EXECUTABLE(KinectOneStream
  src/KinectOneStream.cpp
)

TARGET_LINK_LIBRARIES(KinectOneStream
  ${OpenCV_LIBS}
  ${freenect2_LIBRARIES}
)
