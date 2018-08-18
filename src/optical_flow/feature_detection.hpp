#ifndef PITCHANGLECORRECTION_FEATURES_DETECTION_H
#define PITCHANGLECORRECTION_FEATURES_DETECTION_H

#include <opencv2/opencv.hpp>

namespace pac {

void DetectFeatures(const cv::Mat &grayImage, std::vector<cv::Point2f> &_features);


} // namespace pac

#endif //PITCHANGLECORRECTION_FEATURES_DETECTION_H
