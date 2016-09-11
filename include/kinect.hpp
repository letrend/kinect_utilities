#pragma once
// std
#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>
// freenect
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
// opencv
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

class MyFileLogger: public libfreenect2::Logger
{
private:
    std::ofstream logfile_;
public:
    MyFileLogger(const char *filename)
    {
        if (filename)
            logfile_.open(filename);
        level_ = Debug;
    }
    bool good()
    {
        return logfile_.is_open() && logfile_.good();
    }
    virtual void log(Level level, const std::string &message)
    {
        logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
    }
};

class MyFreenectDevice{
public:
    MyFreenectDevice(){

        filelogger = new MyFileLogger("log.txt");
        if (filelogger->good())
            libfreenect2::setGlobalLogger(filelogger);
        else
            cout << "ERROR: could not initialize logger" << endl;

        pipeline = new libfreenect2::CpuPacketPipeline();

        if(freenect2.enumerateDevices() == 0)
        {
            cout << "ERROR: no device connected!" << endl;
            return;
        }

        string serial = freenect2.getDefaultDeviceSerialNumber();
        dev = freenect2.openDevice(serial, pipeline);

        if(!dev->start())
            cout << "ERROR: could not start kinect device" << endl;

        registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

        undistorted = new libfreenect2::Frame(512, 424, 4);
        registered = new libfreenect2::Frame(512, 424, 4);

        int types = 0;
        types |= libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
        listener = new libfreenect2::SyncMultiFrameListener(types);

        dev->setColorFrameListener(listener);
        dev->setIrAndDepthFrameListener(listener);

        libfreenect2::Freenect2Device::ColorCameraParams cameraParams = dev->getColorCameraParams();
        cout << "cameraparams:\n" << cameraParams.fx << " " << cameraParams.fy << " " << cameraParams.cx << " " << cameraParams.cy << endl;
        libfreenect2::Freenect2Device::IrCameraParams irCameraParams = dev->getIrCameraParams();
        cout << "ircameraparams:\n" << irCameraParams.fx << " " << irCameraParams.fy << " " << irCameraParams.cx << " " << irCameraParams.cy << endl;
    }

    ~MyFreenectDevice(){
        dev->stop();
        dev->close();
        delete registration;
        delete undistorted;
        delete registered;
        delete listener;
        delete filelogger;
    }

    void getVideo(cv::Mat& output) {
        cv::Mat img = cv::Mat(rgb->height,rgb->width,CV_8UC4,rgb->data);
        img.convertTo(output,CV_32FC3);
    }

    void getDepth(cv::Mat& output) {
        output = cv::Mat(undistorted->height,undistorted->width,CV_32FC1,undistorted->data);
        output/=1000.0f; // convert to meter
    }

    void getDepthMM(cv::Mat& output) {
        output = cv::Mat(undistorted->height,undistorted->width,CV_32FC1,undistorted->data);
    }

    void getRgbMapped2Depth(cv::Mat& output) {
        cv::Mat img = cv::Mat(registered->height,registered->width,CV_8UC4,registered->data);
        img.convertTo(output,CV_32FC3);
    }

    bool updateFrames(){
        listener->release(frames);

        if (!listener->waitForNewFrame(frames, 10*1000)){ // wait 10 seconds
            cout << "timeout!" << endl;
            return false;
        }else{
            rgb = frames[libfreenect2::Frame::Color];
            ir = frames[libfreenect2::Frame::Ir];
            depth = frames[libfreenect2::Frame::Depth];
            registration->apply(rgb, depth, undistorted, registered);
            return true;
        }
    }

private:
    MyFileLogger *filelogger;
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    libfreenect2::Registration* registration;
    libfreenect2::Frame *undistorted, *registered;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::FrameMap frames;
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
};