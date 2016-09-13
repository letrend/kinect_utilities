#include <opencv2/opencv.hpp>
#include "kinect.hpp"

int main() {
    MyFreenectDevice device;
    int k;
    cout << "STOP WITH ESC" << endl;
    while(device.updateFrames() && k!=27){
        cv::Mat rgb, rgbMapped2Depth, depth, depthMM;
        device.getVideo(rgb);
        device.getRgbMapped2Depth(rgbMapped2Depth);
        device.getDepth(depth);
        device.getDepthMM(depthMM);

        imshow("rgb", rgb);
        imshow("rgbMapped2Depth", rgbMapped2Depth);
        imshow("depth", depth);
        imshow("depthMM", depthMM);
        k = cv::waitKey(1);
    }
    return 0;
}