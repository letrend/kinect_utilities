#include <opencv2/opencv.hpp>
#include "kinect.hpp"

int main() {
    MyFreenectDevice device;
    int k;
    while(device.updateFrames() && k!=30){
        cv::Mat rgb, rgbMapped2Depth, depth, depthMM;
        device.getVideo(rgb);
        device.getRgbMapped2Depth(rgbMapped2Depth);
        device.getDepth(depth);
        device.getDepthMM(depthMM);

        imshow("rgb", rgb/255.0f);
        imshow("rgbMapped2Depth", rgbMapped2Depth/255.0f);
        imshow("depth", depth);
        imshow("depthMM", depthMM);
        k = cv::waitKey(1);
    }
    return 0;
}