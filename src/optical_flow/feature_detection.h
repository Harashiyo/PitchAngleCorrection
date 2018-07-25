//
// Created by Haraoka Shohei on 2018/07/25.
//

#ifndef PITCHANGLECORRECTION_FEATURES_DETECTION_H
#define PITCHANGLECORRECTION_FEATURES_DETECTION_H

#include <opencv2/opencv.hpp>

namespace pitchanglecorrection {
namespace opticalflow {

void DetectFeatures(cv::Mat &grayImage, std::vector<cv::Point2f> &output);

}
}

#endif //PITCHANGLECORRECTION_FEATURES_DETECTION_H
