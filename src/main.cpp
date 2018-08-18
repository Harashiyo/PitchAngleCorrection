#include "image/image_io.hpp"
#include "optical_flow/optical_flow.hpp"
#include "geometry/motion_estimation.hpp"
#include "geometry/geometry.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace pac;

int main(int argc, char *argv[]) {
    if (argc < 1) {
        cout << "usage: ./a.out [images directory path]" << endl;
    }
    vector<string> files;
    SearchDir(argv[1], files);

    const int interval = 6;

    deque<Mat> frames;
    for (int i = 0; i < interval; i++) {
        frames.push_back(readColorImage(files[i]));
    }
    for (int i = interval; i < files.size(); i++) {
        std::vector<cv::Point2f> prevFeatures;
        std::vector<cv::Point2f> currFeatures;
        CalcOpticalFlowMultFrames(frames, prevFeatures, currFeatures);
        cv::Mat result = frames.back().clone();
        //DrawOpticalFlow(frames.back(),prevFeatures,currFeatures,STRAIGHT_LINE,result);
        //Point2f eof;
        //CalcFocusOfExpansion(frames.back(),prevFeatures,currFeatures,eof);
        //circle(result,eof,8,Scalar(255,0,0),6);
        std::vector<cv::Point2f> maskedPrevFeatures;
        std::vector<cv::Point2f> maskedCurrFeatures;
        double pitch;
        EstimateMotion(prevFeatures, currFeatures,maskedPrevFeatures,maskedCurrFeatures,pitch);
        DrawOpticalFlow(frames.back(),maskedPrevFeatures,maskedCurrFeatures,STRAIGHT_LINE,result);
        showImage(result);
        frames.pop_front();
        frames.push_back(readColorImage(files[i]));
    }
    return 0;
}

