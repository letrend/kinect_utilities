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

    ICPCUDA(size_t pWidth, size_t pHeight, float cx, float cy, float fx, float fy){
        icpOdom = new ICPOdometry(pWidth, pHeight, cx, cy, fx, fy);
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
        // ICP
        icpOdom->initICPModel((unsigned short *)depth0.data, 20.0f);

        icpOdom->initICP((unsigned short *)depth1.data, 20.0f);

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

