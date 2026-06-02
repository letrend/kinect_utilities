#pragma once

#include "ICPOdometry.h"
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

struct ICPResult {
    bool ok = false;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d previousPose = Eigen::Matrix4d::Identity();
    float residual = std::numeric_limits<float>::infinity();
    float inliers = 0.0f;
    float inlierRatio = 0.0f;
    float translationStep = 0.0f;
    float rotationStepDeg = 0.0f;
    float elapsedMs = 0.0f;
    std::string rejectionReason;
};

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
        pose = pose_init;
        T_current = Sophus::SE3d(pose_init);
    }

    uint64_t getCurrTime(){
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    }

    ICPResult getPoseFromDepth(cv::Mat &depth0, cv::Mat &depth1,
                               float depthCutoffM = 20.0f,
                               int iter0 = 10, int iter1 = 5, int iter2 = 4){ ;
        ICPResult result;
        result.previousPose = pose;
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
        icpOdom->setIterations(iter0, iter1, iter2);
        icpOdom->initICPModel((unsigned short *)d0u.data, depthCutoffM);

        icpOdom->initICP((unsigned short *)d1u.data, depthCutoffM);

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
        result.pose = pose;
        result.residual = icpOdom->lastError;
        result.inliers = icpOdom->lastInliers;
        result.inlierRatio = float(icpOdom->lastInliers) / float(std::max<size_t>(1, width * height));
        Eigen::Vector3d dt = pose.topRightCorner(3,1) - result.previousPose.topRightCorner(3,1);
        result.translationStep = (float)dt.norm();
        Eigen::Matrix3d dR = result.previousPose.topLeftCorner(3,3).transpose() * pose.topLeftCorner(3,3);
        Eigen::AngleAxisd aa(dR);
        result.rotationStepDeg = (float)(std::fabs(aa.angle()) * 180.0 / 3.14159265358979323846);
        result.elapsedMs = float(tock - tick) / 1000.0f;
        result.ok = std::isfinite(result.residual) &&
                    std::isfinite(result.translationStep) &&
                    std::isfinite(result.rotationStepDeg) &&
                    result.inliers > 0.0f;
        if (!result.ok)
            result.rejectionReason = "non-finite ICP result";
        return result;
    };
public:
    Eigen::Matrix4d getPose(){
        return pose;
    }
    Eigen::Matrix4d getPose_inv(){
        return pose.inverse();
    };
    void setPose(const Eigen::Matrix4d &newPose) {
        pose = newPose;
        T_current = Sophus::SE3d(newPose);
    }

    float mean_time = 0.0f;
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
