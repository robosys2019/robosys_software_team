#ifndef KINECTONESTREAM_H
#define KINECTONESTREAM_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

libfreenect2::Freenect2 freenect2;
libfreenect2::FrameMap frames;
libfreenect2::Freenect2Device *dev = 0;
libfreenect2::PacketPipeline *pipeline = 0;
libfreenect2::Registration *registration;
libfreenect2::SyncMultiFrameListener *listener;
libfreenect2::Frame *undistorted;

cv::Mat rgbmat, depthmatUndistorted;

ros::NodeHandle *nh;
image_transport::ImageTransport *it;
image_transport::Publisher rgb_pub;
image_transport::Publisher depth_undistorted_pub;

sensor_msgs::ImagePtr rgb_msg;
sensor_msgs::ImagePtr depth_undistorted_msg;


void open_kinect();
void setup_kinect();
void setup_ros();
void loop();
void publish();

#endif //KINECTONESTREAM_H
