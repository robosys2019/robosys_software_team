/*
-- Georgia Tech 2016 Spring
--
-- This is a sample code to show how to use the libfreenet2 with OpenCV
--
-- The code will streams RGB, IR and Depth images from an Kinect sensor.
-- To use multiple Kinect sensor, simply initial other "listener" and "frames"
-- This code refered from sample code provided from libfreenet2: Protonect.cpp
-- https://github.com/OpenKinect/libfreenect2
-- and another discussion from: http://answers.opencv.org/question/76468/opencvkinect-onekinect-for-windows-v2linuxlibfreenect2/
-- Contact: Chih-Yao Ma at <cyma@gatech.edu>
*/

#include "KinectOneStream.h"


bool open_kinect(){
    ROS_INFO("Attempting to connect to Kinect One sensor!");

    if(freenect2.enumerateDevices() == 0){
        ROS_FATAL("No device connected!");
        return -1;
    }

    std::string serial = freenect2.getDefaultDeviceSerialNumber();

    ROS_INFO("SERIAL: %s", serial.c_str());

    if(pipeline){
        dev = freenect2.openDevice(serial, pipeline);
    }else{
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0){
        ROS_FATAL("failure opening device!");
        return -1;
    }

    return 1;
}

void setup_kinect(){
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color |
                                                        libfreenect2::Frame::Depth |
                                                        libfreenect2::Frame::Ir);

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);


    dev->start();
    ROS_INFO("Device serial: %s", dev->getSerialNumber().c_str());
    ROS_INFO("Device firmware: %s", dev->getFirmwareVersion().c_str());


    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    undistorted = new libfreenect2::Frame(512, 424, 4);
    registered = new libfreenect2::Frame(512, 424, 4);
    depth2rgb = new libfreenect2::Frame(1920, 1080 + 2, 4);

}

void setup_ros(){

}

void loop(){
    while(ros::ok())
    {
        listener->waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

        cv::imshow("rgb", rgbmat);
        cv::imshow("ir", irmat / 4096.0f);
        cv::imshow("depth", depthmat / 4096.0f);

        //! [registration]
        registration->apply(rgb, depth, undistorted, registered, true, depth2rgb);

        cv::Mat(undistorted->height, undistorted->width, CV_32FC1, undistorted->data).copyTo(depthmatUndistorted);
        cv::Mat(registered->height, registered->width, CV_8UC4, registered->data).copyTo(rgbd);
        cv::Mat(depth2rgb->height, depth2rgb->width, CV_32FC1, depth2rgb->data).copyTo(rgbd2);


        cv::imshow("undistorted", depthmatUndistorted / 4096.0f);
        cv::imshow("registered", rgbd);
        cv::imshow("depth2RGB", rgbd2 / 4096.0f);

        publish();
        ros::spinOnce();

        int key = cv::waitKey(1);

        listener->release(frames);
    }
}

void publish(){

}

int main(int argc, char **argv){

    ros::init(argc, argv, "kinect");

    if(!open_kinect()){
        return -1;
    }

    setup_kinect();

    setup_ros();

    loop();

    dev->stop();
    dev->close();

    delete registration;
    delete listener;
    delete undistorted;
    delete registered;
    delete depth2rgb;

    ROS_INFO("Streaming Ends!");
    return 0;
}
