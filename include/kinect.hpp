#pragma once
// std
#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <stdexcept>
// freenect
#include <libfreenect2/config.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
// opencv
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

class MyFileLogger : public libfreenect2::Logger {
private:
    std::ofstream logfile_;
public:
    MyFileLogger(const char *filename) {
        if (filename)
            logfile_.open(filename);
        level_ = Debug;
    }

    bool good() {
        return logfile_.is_open() && logfile_.good();
    }

    virtual void log(Level level, const std::string &message) {
        logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
    }
};

class MyFreenectDevice {
public:
    MyFreenectDevice()
        : dev(nullptr), pipeline(nullptr), registration(nullptr),
          undistorted(nullptr), registered(nullptr), listener(nullptr),
          rgb(nullptr), ir(nullptr), depth(nullptr) {
        rgb = frames[libfreenect2::Frame::Color];
        ir = frames[libfreenect2::Frame::Ir];
        depth = frames[libfreenect2::Frame::Depth];

        // Prefer fast GPU pipelines. CpuPacketPipeline is too slow for
        // Kinect v2 and the listener starves. Try OpenGL first because the
        // OpenCL path can hang for seconds on systems with a broken ICD
        // (e.g. beignet on Intel).
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
        try { pipeline = new libfreenect2::CudaPacketPipeline();
              cout << "[kinect] using CudaPacketPipeline" << endl; }
        catch (...) { pipeline = nullptr; }
#endif
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        if (!pipeline) {
            try { pipeline = new libfreenect2::OpenGLPacketPipeline();
                  cout << "[kinect] using OpenGLPacketPipeline" << endl; }
            catch (...) { pipeline = nullptr; }
        }
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if (!pipeline) {
            try { pipeline = new libfreenect2::OpenCLPacketPipeline();
                  cout << "[kinect] using OpenCLPacketPipeline" << endl; }
            catch (...) { pipeline = nullptr; }
        }
#endif
        if (!pipeline) {
            pipeline = new libfreenect2::CpuPacketPipeline();
            cout << "[kinect] WARNING: falling back to CpuPacketPipeline (slow)" << endl;
        }

        if (freenect2.enumerateDevices() == 0) {
            cout << "ERROR: no device connected!" << endl;
            throw std::runtime_error("no Kinect device connected");
        }

        string serial = freenect2.getDefaultDeviceSerialNumber();
        dev = freenect2.openDevice(serial, pipeline);

        if (!dev || !dev->start())
            throw std::runtime_error("could not start Kinect device");

        registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

        int types = 0;
        types |= libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
        listener = new libfreenect2::SyncMultiFrameListener(types);

        dev->setColorFrameListener(listener);
        dev->setIrAndDepthFrameListener(listener);

        irCameraParams = dev->getIrCameraParams();
        cout << "IR camera params: \n"
             << "fx: " << irCameraParams.fx << "\n"
             << "fy: " << irCameraParams.fy << "\n"
             << "cx: " << irCameraParams.cx << "\n"
             << "cy: " << irCameraParams.cy << endl;

        undistorted = new libfreenect2::Frame(512, 424, 4);
        registered = new libfreenect2::Frame(512, 424, 4);

        K << irCameraParams.fx, 0, irCameraParams.cx,
                0, irCameraParams.fy, irCameraParams.cy,
                0, 0, 1;
        Kinv = K.inverse();
    }

    ~MyFreenectDevice() {
        if (dev) {
            dev->stop();
            dev->close();
        }
        delete registration;
        delete undistorted;
        delete registered;
        delete listener;
        delete pipeline;
    }

    void getVideo(cv::Mat &output) {
//        cv::Mat img = cv::Mat(rgb->height,rgb->width,CV_8UC4,rgb->data), img2;
        //        cv::flip(img, img, 1);
        output = cv::Mat(rgb->height, rgb->width, CV_8UC3);
        uchar *aOut = (uchar *) output.data;
        for (int y = 0; y < rgb->height; y++) {
            for (int x = 0; x < rgb->width; x++) {
                for (int c = 0; c < 3; c++) {
                    aOut[c + 3 * (x + (size_t) rgb->width * y)] = (uchar) rgb->data[c +
                                                                                    4 * (x + (size_t) rgb->width * y)];
                }
            }
        }
    }

    void getDepth(cv::Mat &output) {
        cv::Mat depth = cv::Mat(undistorted->height, undistorted->width, CV_32FC1, undistorted->data);
        depth /= 1000.0f;
        depth.convertTo(output, CV_16U);
        cv::flip(output, output, 1);
    }

    void getDepthMM(cv::Mat &output) {
        cv::Mat depth = cv::Mat(undistorted->height, undistorted->width, CV_32FC1, undistorted->data);
        depth.copyTo(output);
        cv::flip(output, output, 1);
    }

    void getRgbMapped2Depth(cv::Mat &output) {
        output = cv::Mat(registered->height, registered->width, CV_32FC3);
        float *aOut = (float *) output.data;
        for (int y = 0; y < registered->height; y++) {
            for (int x = 0; x < registered->width; x++) {
                for (int c = 0; c < 3; c++) {
                    aOut[c + 3 * (x + (size_t) registered->width * y)] =
                            (float) registered->data[c + 4 * (x + (size_t) registered->width * y)] / 255.0f;
                }
            }
        }
        cv::flip(output, output, 1);
    }

    bool updateFrames() {
        listener->release(frames);

        if (!listener->waitForNewFrame(frames, 2*1000)){ // wait 2 seconds
            cout << "WARNING: kinect timeout!" << endl;
            return false;
        } else {
            rgb = frames[libfreenect2::Frame::Color];
            ir = frames[libfreenect2::Frame::Ir];
            depth = frames[libfreenect2::Frame::Depth];
            registration->apply(rgb, depth, undistorted, registered);
            return true;
        }
    }

    Vector3d projectTo3D(float x, float y){
        Vector3d point2D(x,y,1);
        Vector3d point3D = Kinv*point2D;
        return point3D;
    }

    libfreenect2::Freenect2Device::IrCameraParams irCameraParams;
private:
    Matrix3d K, Kinv;
    MyFileLogger *filelogger;
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev;
    libfreenect2::PacketPipeline *pipeline;
    libfreenect2::Registration *registration;
    libfreenect2::Frame *undistorted, *registered;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::FrameMap frames;
    libfreenect2::Frame *rgb;
    libfreenect2::Frame *ir;
    libfreenect2::Frame *depth;
};
