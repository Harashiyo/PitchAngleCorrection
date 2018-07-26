#ifndef PITCHANGLECORRECTION_OPTICAL_FLOW_H
#define PITCHANGLECORRECTION_OPTICAL_FLOW_H

#include "feature_detection.hpp"
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
    enum LineType {
        STRAIGHT_LINE,
        LINE_SEGMENT
    };
    OpticalFlow(cv::Mat &image);
    void CalcOpticalFlow(cv::Mat &image,std::vector< std::vector<cv::Point2f>> &output);
    void DrawOpticalFlow(cv::Mat &image, LineType l, cv::Mat &output);
    void PrintFeatures(cv::Mat &image,cv::Mat &output);
};



}
}

#endif //PITCHANGLECORRECTION_OPTICAL_FLOW_H
