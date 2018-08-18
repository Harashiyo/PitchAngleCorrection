//
// Created by Haraoka Shohei on 2018/08/09.
//

#ifndef PITCHANGLECORRECTION_EPIPOLAR_GEOMETRY_HPP
#define PITCHANGLECORRECTION_EPIPOLAR_GEOMETRY_HPP

#include <opencv2/opencv.hpp>

namespace pac {

float SolveY(const cv::Vec3f &efficient, float x);

float SolveX(const cv::Vec3f &efficient, float y);

void DrawPoints(const cv::Mat &image, const std::vector<cv::Point2f> &points, cv::Mat &_result, int radius = 4,
                const cv::Scalar &color = cv::Scalar(255, 0, 255));

void DrawLines(const cv::Mat &image, const std::vector<cv::Vec3f> &lines, cv::Mat &_result, int thickness = 4,
               const cv::Scalar &color = cv::Scalar(0, 0, 255));

void DrawLines(const cv::Mat &image, const std::vector<cv::Vec3f> &lines, cv::Mat &_result, const cv::Point2f &translate,
               int thickness = 2, const cv::Scalar &color = cv::Scalar(0, 0, 255));

cv::Vec3f CalcLine(const cv::Point2f &point1, const cv::Point2f &point2);

void CalcLines(const std::vector<cv::Point2f> &point1, const std::vector<cv::Point2f> &point2,
               std::vector<cv::Vec3f> &_lines);

void DrawEpipolarLines(const cv::Mat &image, const std::vector<cv::Point2f> &points, int whichImage,
                       const cv::Mat &fundamentalMat, cv::Mat &_result,
                       const cv::Point2f &translate = cv::Point2f(0.0, 0.0));

float CalcDistance(const cv::Vec3f &line, const cv::Point2f &point);

} // namespace pac


#endif //PITCHANGLECORRECTION_EPIPOLAR_GEOMETRY_HPP
