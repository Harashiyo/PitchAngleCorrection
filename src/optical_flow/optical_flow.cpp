#include "optical_flow.hpp"

namespace pitchanglecorrection {
namespace opticalflow {

OpticalFlow::OpticalFlow(cv::Mat &image) {
    if (image.channels() == 1) {
        currFrameGray_ = image.clone();
    } else {
        cv::cvtColor(image, currFrameGray_, cv::COLOR_BGR2GRAY);
    }
    DetectFeatures(currFrameGray_, currFeatures_);
}

void OpticalFlow::CalcOpticalFlow(cv::Mat &image, std::vector<std::vector<cv::Point2f>> &output) {
    prevFrameGray_ = currFrameGray_.clone();
    prevFeatures_.clear();
    std::copy(currFeatures_.begin(), currFeatures_.end(), std::back_inserter(prevFeatures_));
    if (image.channels() == 1) {
        currFrameGray_ = image.clone();
    } else {
        cv::cvtColor(image, currFrameGray_, cv::COLOR_BGR2GRAY);
    }
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
        if (!featuresFound[i]) {
            continue;
        }
        std::vector<cv::Point2f> pair;
        pair.push_back(prevFeatures_[i]);
        pair.push_back(currFeatures_[i]);
        result_.push_back(pair);
    }
    output = result_;
}

void OpticalFlow::DrawOpticalFlow(cv::Mat &image, LineType l, cv::Mat &output) {
    output = image.clone();
    if (l == LINE_SEGMENT) {
        for (int i = 0; i < result_.size(); i++) {
            cv::Point p1 = cv::Point((int) result_[i][0].x, (int) result_[i][0].y);
            cv::Point p2 = cv::Point((int) result_[i][1].x, (int) result_[i][1].y);
            line(output, p1, p2, cv::Scalar(0, 0, 255), 12);
        }
    } else if (l == STRAIGHT_LINE) {
        for (int i = 0; i < result_.size(); i++) {
            float slope = (result_[i][1].y - result_[i][0].y) / (result_[i][1].x - result_[i][0].x);
            float intercept = result_[i][0].y - slope * result_[i][0].x;
            float y1, y2, x1, x2;

            if (intercept < 0) {
                y1 = 0;
                x1 = -intercept / slope;
            } else if (intercept >= output.rows) {
                y1 = output.rows - 1;
                x1 = (y1 - intercept) / slope;
            } else {
                y1 = intercept;
                x1 = 0;
            }
            y2 = slope * (output.cols - 1) + intercept;
            if (y2 < 0) {
                y2 = 0;
                x2 = -intercept / slope;
            } else if (y2 >= output.rows) {
                y2 = output.rows - 1;
                x2 = (y2 - intercept) / slope;
            } else {
                x2 = output.cols - 1;
            }
            cv::Point p1 = cv::Point((int) x1, (int) y1);
            cv::Point p2 = cv::Point((int) x2, (int) y2);
            cv::line(output, p1, p2, cv::Scalar(0, 0, 255), 4);
        }
    }
}

void OpticalFlow::PrintFeatures(cv::Mat &image, cv::Mat &output) {
    output = image.clone();
    const int r = 4;
    for (int i = 0; i < prevFeatures_.size(); i++) {
        cv::circle(output, prevFeatures_[i], r, cv::Scalar(255, 0, 255), -1, 24, 0);
    }
    for (int i = 0; i < currFeatures_.size(); i++) {
        cv::circle(output, currFeatures_[i], r, cv::Scalar(0, 255, 255), -1, 24, 0);
    }
}

}
}
