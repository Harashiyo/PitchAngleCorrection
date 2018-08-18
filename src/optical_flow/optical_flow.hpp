#ifndef PITCHANGLECORRECTION_OPTICAL_FLOW_H
#define PITCHANGLECORRECTION_OPTICAL_FLOW_H

#include "feature_detection.hpp"
#include "../geometry/geometry.hpp"
#include <opencv2/opencv.hpp>


namespace pac {

enum LineType {
    STRAIGHT_LINE,
    LINE_SEGMENT
};

void CalcOpticalFlow(const cv::Mat &prevImage, const cv::Mat &currImage,
                     const std::vector<cv::Point2f> &prevFeatures, std::vector<cv::Point2f> &_currFeatures,
                     std::vector<uchar> &_featuresFound);


void CalcOpticalFlowTwoFrames(const cv::Mat &prevImage, const cv::Mat &currImage,
                              std::vector<cv::Point2f> &_prevFeatures,
                              std::vector<cv::Point2f> &_currFeatures);

void CalcOpticalFlowMultFrames(const std::deque<cv::Mat> &images, std::vector<cv::Point2f> &_prevFeaturesFound,
                               std::vector<cv::Point2f> &_currFeaturesFound);

void DrawOpticalFlow(const cv::Mat &image, const std::vector<cv::Point2f> &prevFeatures,
                     const std::vector<cv::Point2f> &currFeatures, LineType l, cv::Mat &_result, int thickness = 4,
                     const cv::Scalar &color = cv::Scalar(0, 0, 255));

void Normalization(const cv::Mat &image, const std::vector<cv::Point2f> &prevFeatures,
                   const std::vector<cv::Point2f> &currFeatures, std::vector<cv::Point2f> &_prevNormalized,
                   std::vector<cv::Point2f> &_currNormalized);

void Normalization2(const cv::Mat &image, const std::vector<cv::Point2f> &prevFeatures,
                    const std::vector<cv::Point2f> &currFeatures, std::vector<cv::Point2f> &_prevNormalized,
                    std::vector<cv::Point2f> &_currNormalized);

void LineFilter(const std::vector<cv::Vec3f> &lines,const cv::Point2f &upperLeft,const cv::Point2f &bottomRight,
                std::vector<cv::Vec3f> &_result);

void CalcFocusOfExpansion(const cv::Mat &image, const std::vector<cv::Point2f> &prevFeatures,
                          const std::vector<cv::Point2f> &currFeatures, cv::Point2f &_eof);

} // namespace pac

#endif //PITCHANGLECORRECTION_OPTICAL_FLOW_H
