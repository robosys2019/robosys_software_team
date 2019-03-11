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
libfreenect2::Freenect2Device *dev = 0;
libfreenect2::PacketPipeline *pipeline = 0;
libfreenect2::Registration *registration;
libfreenect2::FrameMap frames;
libfreenect2::SyncMultiFrameListener *listener;
libfreenect2::Frame *undistorted, *registered, *depth2rgb;

cv::Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;


ros:: NodeHandle nh;
ros::Publisher rgb_pub;
ros::Publisher depth_pub;
ros::Publisher dept_undistorted_pub;
ros::Publisher ir_pub;
ros::Publisher rgbd_pub;


bool open_kinect();
void setup_kinect();
void setup_ros();
void loop();
void publish();

#endif //KINECTONESTREAM_H
