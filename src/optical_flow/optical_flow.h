//
// Created by Haraoka Shohei on 2018/07/25.
//

#ifndef PITCHANGLECORRECTION_OPTICAL_FLOW_H
#define PITCHANGLECORRECTION_OPTICAL_FLOW_H

#include "feature_detection.h"
#include <opencv2/opencv.hpp>

namespace pitchanglecorrection {
namespace opticalflow {

class OpticalFlow {
private:
    cv::Mat prevFrameGray_;
    cv::Mat currFrameGray_;
    std::vector<cv::Point2f> prevFeatures_;
    std::vector<cv::Point2f> currFeatures_;
    std::vector< std::vector<cv::Point2f>> result_;
public:
    OpticalFlow(cv::Mat &image);

    std::vector< std::vector<cv::Point2f>> CalcOpticalFlow(cv::Mat &image);

    cv::Mat DrawOpticalFlow(cv::Mat &image, int option);

    cv::Mat PrintFeatures(cv::Mat &image);
};



}
}

#endif //PITCHANGLECORRECTION_OPTICAL_FLOW_H
