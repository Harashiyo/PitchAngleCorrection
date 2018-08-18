//
// Created by Haraoka Shohei on 2018/08/09.
//

#include "geometry.hpp"

namespace pac {

float SolveY(const cv::Vec3f &efficient, float x) {
    return -(efficient[0] * x + efficient[2]) / efficient[1];
}

float SolveX(const cv::Vec3f &efficient, float y) {
    return -(efficient[1] * y + efficient[2]) / efficient[0];
}

void DrawLines(const cv::Mat &image, const std::vector<cv::Vec3f> &lines, cv::Mat &_result, int thickness,
               const cv::Scalar &color) {
    _result = image.clone();
    for (cv::Vec3f l : lines) {
        if (l[1]) {
            // Y軸と交わる直線の場合
            float width = image.size().width;
            cv::Point2f left = cv::Point2f(0.0, SolveY(l, 0.0));
            cv::Point2f right = cv::Point2f(width, SolveY(l, width));
            cv::line(_result, left, right, color, thickness);
        } else {
            // Y軸に平行な直線の場合
            float x = -l[2] / l[0];
            cv::Point2f top(x, 0.0);
            cv::Point2f bottom(x, image.size().height);
            cv::line(_result, top, bottom, color, thickness);
        }
    }
    return;
}

void
DrawLines(const cv::Mat &image, const std::vector<cv::Vec3f> &lines, cv::Mat &_result, const cv::Point2f &translate,
          int thickness, const cv::Scalar &color) {
    _result = image.clone();
    for (cv::Vec3f l : lines) {
        if (l[1]) {
            // Y軸と交わる直線の場合
            float width = image.size().width;
            cv::Point2f left = cv::Point2f(0.0, SolveY(l, 0.0 - translate.x) + translate.y);
            cv::Point2f right = cv::Point2f(width, SolveY(l, width - translate.x) + translate.y);
            cv::line(_result, left, right, color, thickness);
        } else {
            // Y軸に平行な直線の場合
            float x = -l[2] / l[0];
            cv::Point2f top = cv::Point2f(x + translate.x, 0.0);
            cv::Point2f bottom = cv::Point2f(x + translate.x, image.size().height);
            cv::line(_result, top, bottom, color, thickness);
        }
    }
    return;
}

void DrawPoints(const cv::Mat &image, const std::vector<cv::Point2f> &points, cv::Mat &_result, int radius,
                const cv::Scalar &color) {
    _result = image.clone();
    for (int i = 0; i < points.size(); i++) {
        cv::circle(_result, points[i], radius, color, -1, cv::LINE_8, 0);
    }
    return;
}

cv::Vec3f CalcLine(const cv::Point2f &point1, const cv::Point2f &point2) {
    float a = point1.y - point2.y;
    float b = point2.x - point1.x;
    float c = point1.x * point2.y - point2.x * point1.y;
    return cv::Vec3f(a, b, c);
}

void CalcLines(const std::vector<cv::Point2f> &point1, const std::vector<cv::Point2f> &point2,
               std::vector<cv::Vec3f> &_lines) {
    std::vector<cv::Vec3f> lines;
    for (int i = 0; i < point1.size(); i++) {
        lines.push_back(CalcLine(point1[i], point2[i]));
    }
    _lines = lines;
}

void DrawEpipolarLines(const cv::Mat &image, const std::vector<cv::Point2f> &points, int whichImage,
                       const cv::Mat &fundamentalMat, cv::Mat &_result,
                       const cv::Point2f &translate) {
    float point[points.size() * 2];
    for (int i = 0; i < points.size(); i++) {
        point[i * 2] = points[i].x;
        point[i * 2 + 1] = points[i].y;
    }
    cv::Mat pointsMat = cv::Mat(1, points.size(), CV_32FC2, point);
    std::vector<cv::Vec3f> lines;
    cv::computeCorrespondEpilines(pointsMat, whichImage, fundamentalMat, lines);
    DrawLines(image, lines, _result, translate);
}


float CalcDistance(const cv::Vec3f &line, const cv::Point2f &point) {
    return std::abs(line[0] * point.x + line[1] * point.y + line[2]) / std::sqrt(line[0] * line[0] + line[1] * line[1]);
}


} // namespace pac

