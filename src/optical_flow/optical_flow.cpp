//
// Created by Haraoka Shohei on 2018/07/25.
//

#include "optical_flow.h"

namespace pitchanglecorrection {
namespace opticalflow {

OpticalFlow::OpticalFlow(cv::Mat &image) {
    cv::cvtColor(image, currFrameGray_, cv::COLOR_BGR2GRAY);
    DetectFeatures(currFrameGray_, currFeatures_);
}

std::vector< std::vector<cv::Point2f>> OpticalFlow::CalcOpticalFlow(cv::Mat &image) {
    prevFrameGray_ = currFrameGray_.clone();
    prevFeatures_.clear();
    std::copy(currFeatures_.begin(), currFeatures_.end(), std::back_inserter(prevFeatures_));
    cv::cvtColor(image, currFrameGray_, cv::COLOR_BGR2GRAY);
    currFeatures_.clear();
    DetectFeatures(currFrameGray_, currFeatures_);
    std::vector<uchar> featuresFound;
    std::vector<float> featuresErrors;
    cv::calcOpticalFlowPyrLK(
            prevFrameGray_,
            currFrameGray_,
            prevFeatures_,
            currFeatures_,
            featuresFound,
            featuresErrors);
    result_.clear();
    for (int i = 0; i < featuresFound.size(); i++) {
        if (!featuresFound[i]){
            continue;
        }
        std::vector<cv::Point2f> pair;
        pair.push_back(prevFeatures_[i]);
        pair.push_back(currFeatures_[i]);
        result_.push_back(pair);
    }
    return result_;
}

cv::Mat OpticalFlow::DrawOpticalFlow(cv::Mat &image, int option){
    cv::Mat frame = image.clone();
    if(option == 0){
        for(int i=0;i<result_.size();i++){
            cv::Point p1 = cv::Point((int) result_[i][0].x, (int) result_[i][0].y);
            cv::Point p2 = cv::Point((int) result_[i][1].x, (int) result_[i][1].y);
            line(frame, p1, p2, cv::Scalar(0, 0, 255), 12);
        }
    }else if(option == 1){
        for(int i=0;i<result_.size();i++){
            float slope = (result_[i][1].y - result_[i][0].y) / (result_[i][1].x - result_[i][0].x);
            float intercept = result_[i][0].y - slope * result_[i][0].x;
            float y1, y2, x1, x2;

            if (intercept < 0) {
                y1 = 0;
                x1 = -intercept / slope;
            } else if (intercept >= frame.rows) {
                y1 = frame.rows - 1;
                x1 = (y1 - intercept) / slope;
            } else {
                y1 = intercept;
                x1 = 0;
            }
            y2 = slope * (frame.cols - 1) + intercept;
            if (y2 < 0) {
                y2 = 0;
                x2 = -intercept / slope;
            } else if (y2 >= frame.rows) {
                y2 = frame.rows - 1;
                x2 = (y2 - intercept) / slope;
            } else {
                x2 = frame.cols - 1;
            }
            cv::Point p1 = cv::Point((int) x1, (int) y1);
            cv::Point p2 = cv::Point((int) x2, (int) y2);
            cv::line(frame, p1, p2, cv::Scalar(0, 0, 255), 4);
        }
    }
    return frame;

}

cv::Mat OpticalFlow::PrintFeatures(cv::Mat &image){
    cv::Mat frame = image.clone();
    const int r = 4;
    for (int i = 0; i < currFeatures_.size(); i++) {
        cv::circle(frame, currFeatures_[i], r, cv::Scalar(0,255,255), -1, 24, 0);
    }
    return frame;
}

}
}
