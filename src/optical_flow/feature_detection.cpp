//
// Created by Haraoka Shohei on 2018/07/25.
//

#include "feature_detection.h"

namespace pitchanglecorrection {
namespace opticalflow {

void DetectCorners(cv::Mat &grayImage, std::vector<cv::Point2f> &output) {
    const int kMaxCorners = 100;
    const double kQualityLevel = 0.05;
    const double kMinDistance = 10;
    const int kBlockSize = 3;
    const bool kUseHarrisDetector = true;
    const double kK = 0.04;

    cv::goodFeaturesToTrack(grayImage,
                            output,
                            kMaxCorners,
                            kQualityLevel,
                            kMinDistance,
                            cv::Mat(),
                            kBlockSize,
                            kUseHarrisDetector,
                            kK);
    cv::cornerSubPix(grayImage, output, cv::Size(21, 21), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01));
}

void DetectFeatures(cv::Mat &grayImage, std::vector<cv::Point2f> &output) {
    const int kSplitNumber = 3;
    const int kSplitHeight = grayImage.rows / kSplitNumber;

    for (int i = 0; i < kSplitNumber; i++) {
        cv::Rect roi(0, kSplitHeight * i, grayImage.cols, kSplitHeight);
        cv::Mat splitGrayImage = grayImage(roi);
        std::vector<cv::Point2f> corners;
        DetectCorners(splitGrayImage, corners);
        if (i != 0) {
            for (int j = 0; j < corners.size(); j++) {
                corners[j].y += kSplitHeight * i;
            }
        }
        output.insert(output.end(), corners.begin(), corners.end());
    }
}
}
}