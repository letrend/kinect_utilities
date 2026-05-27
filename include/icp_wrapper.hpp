#pragma once

#include "ICPOdometry.h"
#include <chrono>

class ICPCUDA{
public:
    ICPCUDA(int pWidth, int pHeight, Eigen::Matrix4d pose_init, float cx, float cy, float fx, float fy){
        icpOdom = new ICPOdometry(pWidth, pHeight, cx, cy, fx, fy);
        pose = pose_init;
        T_current = Sophus::SE3d(pose_init);
        depth0 = cv::Mat::zeros(pHeight, pWidth, CV_16U);
        depth1 = cv::Mat::zeros(pHeight, pWidth, CV_16U);
        width = pWidth;
        height = pHeight;
    };

    ICPCUDA(size_t pWidth, size_t pHeight, float cx, float cy, float fx, float fy,
            float distThresh = 0.10f,
            float angleThresh = 0.342020143f /* sin(20 deg) */){
        icpOdom = new ICPOdometry(pWidth, pHeight, cx, cy, fx, fy, distThresh, angleThresh);
        pose = Eigen::Matrix4d::Identity();
        T_current = Sophus::SE3d(pose);
        depth0 = cv::Mat::zeros(pHeight, pWidth, CV_16U);
        depth1 = cv::Mat::zeros(pHeight, pWidth, CV_16U);
        width = pWidth;
        height = pHeight;
    };

    void setInitialPose(Eigen::Matrix4d pose_init){
        T_current = Sophus::SE3d(pose_init);
    }

    uint64_t getCurrTime(){
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    }

    void getPoseFromDepth(cv::Mat &depth0, cv::Mat &depth1){ ;
        // ICPOdometry expects 16-bit unsigned depth in millimeters. Convert
        // any input format to CV_16U so the byte layout actually matches what
        // the CUDA kernels read. (Reinterpreting CV_32F bytes as uint16 yields
        // garbage and the ICP returns an essentially identity transform.)
        auto toU16mm = [](const cv::Mat &in, cv::Mat &out) {
            if (in.type() == CV_16U) { out = in; return; }
            if (in.type() == CV_32F) {
                in.convertTo(out, CV_16U);   // already in mm in this codebase
                return;
            }
            if (in.type() == CV_16S) { in.convertTo(out, CV_16U); return; }
            // Fallback: try a direct conversion.
            in.convertTo(out, CV_16U);
        };
        cv::Mat d0u, d1u;
        toU16mm(depth0, d0u);
        toU16mm(depth1, d1u);
        // ICP
        icpOdom->initICPModel((unsigned short *)d0u.data, 20.0f);

        icpOdom->initICP((unsigned short *)d1u.data, 20.0f);

        T_prev = T_current;

        Sophus::SE3d T_prev_curr = T_prev.inverse() * T_current;

        uint64_t tick = getCurrTime();

        icpOdom->getIncrementalTransformation(T_prev_curr, 288, 32);

        uint64_t tock = getCurrTime();

        mean_time = (float(count) * mean_time + (tock - tick) / 1000.0f) / float(count + 1);
        count++;

        T_current = T_prev * T_prev_curr;

        pose.topLeftCorner(3, 3) = T_current.rotationMatrix();
        pose.topRightCorner(3, 1) = T_current.translation();
    };
public:
    Eigen::Matrix4d getPose(){
        return pose;
    }
    Eigen::Matrix4d getPose_inv(){
        return pose.inverse();
    };

    float mean_time;
private:
    ICPOdometry *icpOdom;
    Eigen::Matrix4d pose;
    Sophus::SE3d T_current, T_prev;
    cv::Mat depth0;
    cv::Mat depth1;
    std::vector<Eigen::Matrix< double, 3, 1 >> translations;
    size_t width,height;

    uint64_t count = 0;
};

