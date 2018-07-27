#include "optical_flow.hpp"

namespace pitchanglecorrection {
namespace opticalflow {

OpticalFlow::OpticalFlow(cv::Mat &image) {
    if (image.channels() == 1) {
        prevFrameGray_ = image.clone();
    } else {
        cv::cvtColor(image, prevFrameGray_, cv::COLOR_BGR2GRAY);
    }
    DetectFeatures(prevFrameGray_, prevFeatures_);
}

void OpticalFlow::CalcOpticalFlow(cv::Mat &image, std::vector<std::vector<cv::Point2f>> &output) {
    cv::Mat currFrameGray;
    if (image.channels() == 1) {
        currFrameGray = image.clone();
    } else {
        cv::cvtColor(image, currFrameGray, cv::COLOR_BGR2GRAY);
    }

    std::vector<cv::Point2f> nextPts;
    std::vector<uchar> featuresFound;
    std::vector<float> featuresErrors;
    const cv::Size kWinSize = cv::Size(21, 21);
    const int kMaxLevel = 3;
    const cv::TermCriteria kCriteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
    const int kFlags = 0;
    const double kMinEigThreshold = 1e-4;
    // Parameters:
    //      prevImg	            first 8-bit input image or pyramid constructed by buildOpticalFlowPyramid.
    //      nextImg	            second input image or pyramid of the same size and the same type as prevImg.
    //      prevPts	            vector of 2D points for which the flow needs to be found; point coordinates must be single-precision floating-point numbers.
    //      nextPts	            output vector of 2D points (with single-precision floating-point coordinates) containing the calculated new positions of input features in the second image; when OPTFLOW_USE_INITIAL_FLOW flag is passed, the vector must have the same size as in the input.
    //      status	            output status vector (of unsigned chars); each element of the vector is set to 1 if the flow for the corresponding features has been found, otherwise, it is set to 0.
    //      err	                output vector of errors; each element of the vector is set to an error for the corresponding feature, type of the error measure can be set in flags parameter; if the flow wasn't found then the error is not defined (use the status parameter to find such cases).
    //      winSize	            size of the search window at each pyramid level.
    //      maxLevel	        0-based maximal pyramid level number; if set to 0, pyramids are not used (single level), if set to 1, two levels are used, and so on; if pyramids are passed to input then algorithm will use as many levels as pyramids have but no more than maxLevel.
    //      criteria	        parameter, specifying the termination criteria of the iterative search algorithm (after the specified maximum number of iterations criteria.maxCount or when the search window moves by less than criteria.epsilon.
    //      flags	            operation flags:
    //                              OPTFLOW_USE_INITIAL_FLOW uses initial estimations, stored in nextPts; if the flag is not set, then prevPts is copied to nextPts and is considered the initial estimate.
    //                              OPTFLOW_LK_GET_MIN_EIGENVALS use minimum eigen values as an error measure (see minEigThreshold description); if the flag is not set, then L1 distance between patches around the original and a moved point, divided by number of pixels in a window, is used as a error measure.
    //      minEigThreshold     the algorithm calculates the minimum eigen value of a 2x2 normal matrix of optical flow equations (this matrix is called a spatial gradient matrix in [20]), divided by number of pixels in a window; if this value is less than minEigThreshold, then a corresponding feature is filtered out and its flow is not processed, so it allows to remove bad points and get a performance boost.
    cv::calcOpticalFlowPyrLK(
            prevFrameGray_,
            currFrameGray,
            prevFeatures_,
            nextPts,
            featuresFound,
            featuresErrors,
            kWinSize,
            kMaxLevel,
            kCriteria,
            kFlags,
            kMinEigThreshold);
    result_.clear();
    for (int i = 0; i < featuresFound.size(); i++) {
        if (!featuresFound[i]) {
            continue;
        }
        std::vector<cv::Point2f> pair;
        pair.push_back(prevFeatures_[i]);
        pair.push_back(nextPts[i]);
        result_.push_back(pair);
    }
    output = result_;

    prevFrameGray_ = currFrameGray.clone();
    prevFeatures_.clear();
    DetectFeatures(currFrameGray, prevFeatures_);
}

void OpticalFlow::DrawOpticalFlow(cv::Mat &image, LineType l, cv::Mat &output) {
    output = image.clone();
    if (l == LINE_SEGMENT) {
        for (int i = 0; i < result_.size(); i++) {
            cv::Point p1 = cv::Point((int) result_[i][0].x, (int) result_[i][0].y);
            cv::Point p2 = cv::Point((int) result_[i][1].x, (int) result_[i][1].y);
            line(output, p1, p2, cv::Scalar(0, 0, 255), 6);
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
    const int kRadius = 4;
    const cv::Scalar kPrevFeaturesColor(255, 0, 255);
    const cv::Scalar kCurrFeaturesColor(0, 255, 255);
    for (int i = 0; i < result_.size(); i++) {
        cv::circle(output, result_[i][0], kRadius, kPrevFeaturesColor, -1, cv::LINE_8, 0);
        cv::circle(output, result_[i][1], kRadius, kCurrFeaturesColor, -1, cv::LINE_8, 0);
    }
}

}
}
