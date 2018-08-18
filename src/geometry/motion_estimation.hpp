#ifndef PITCHANGLECORRECTION_MOTION_ESTIMATION_HPP
#define PITCHANGLECORRECTION_MOTION_ESTIMATION_HPP

#include "geometry.hpp"
#include "../image/camera.hpp"
#include <opencv2/opencv.hpp>


namespace pac {

void CalcFundamentalMat(const std::vector<cv::Point2f> &points1, const std::vector<cv::Point2f> &points2,
                   std::vector<cv::Point2f> &_maskedPoints1, std::vector<cv::Point2f> &_maskedPoints2,
                   cv::Mat &_fundamentalMat);

void CalcEssentialMat(const cv::Mat &fundamentalMat, const cv::Mat &intrinsicMat, cv::Mat &_essensialMat);

void CalcExtrinsicParameters(const std::vector<cv::Point2f> &points1, const std::vector<cv::Point2f> &points2,
                             const cv::Mat &essentialMat, std::vector<cv::Point2f> &_maskedPoints1,
                             std::vector<cv::Point2f> &_maskedPoints2, cv::Mat &_rotationMat,
                             cv::Mat &_translationVec);

double CalcPitchAngle(const cv::Mat &rotationMat);

void EstimateMotion(const std::vector<cv::Point2f> &points1, const std::vector<cv::Point2f> &points2,
                    std::vector<cv::Point2f> &_maskedPoints1, std::vector<cv::Point2f> &_maskedPoints2,
                    double &_pitch);

} // namespace pac

#endif //PITCHANGLECORRECTION_MOTION_ESTIMATION_HPP
