#include "motion_estimation.hpp"

namespace pac {

void CalcFundamentalMat(const std::vector<cv::Point2f> &points1, const std::vector<cv::Point2f> &points2,
                        std::vector<cv::Point2f> &_maskedPoints1, std::vector<cv::Point2f> &_maskedPoints2,
                        cv::Mat &_fundamentalMat) {
    std::vector<uchar> mask;
    const int method = cv::FM_RANSAC;
    const double param1 = 3;
    const double param2 = 0.99;
    // Parameters:
    //      points1     Array of N points from the first image. The point coordinates should be floating-point (single or double precision).
    //      points2     Array of the second image points of the same size and format as points1 .
    //      method      Method for computing a fundamental matrix.
    //                      CV_FM_7POINT for a 7-point algorithm.  N = 7
    //                      CV_FM_8POINT for an 8-point algorithm.  N \ge 8
    //                      CV_FM_RANSAC for the RANSAC algorithm.  N \ge 8
    //                      CV_FM_LMEDS for the LMedS algorithm.  N \ge 8
    //      param1      Parameter used for RANSAC. It is the maximum distance from a point to an epipolar line in pixels, beyond which the point is considered an outlier and is not used for computing the final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the point localization, image resolution, and the image noise.
    //      param2      Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of confidence (probability) that the estimated matrix is correct.
    //      mask        Output array of N elements, every element of which is set to 0 for outliers and to 1 for the other points. The array is computed only in the RANSAC and LMedS methods. For other methods, it is set to all 1’s.
    _fundamentalMat = cv::findFundamentalMat(points1, points2, mask, method, param1, param2);
    std::vector<cv::Point2f> maskedPoint1;
    std::vector<cv::Point2f> maskedPoint2;
    for (int i = 0; i < mask.size(); i++) {
        if (mask[i]) {
            maskedPoint1.push_back(points1[i]);
            maskedPoint2.push_back(points2[i]);
        }
    }
    _maskedPoints1 = maskedPoint1;
    _maskedPoints2 = maskedPoint2;
}

void CalcEssentialMat(const cv::Mat &fundamentalMat, const cv::Mat &intrinsicMat, cv::Mat &_essensialMat) {
    _essensialMat = intrinsicMat.t() * fundamentalMat * intrinsicMat;
}

void CalcExtrinsicParameters(const std::vector<cv::Point2f> &points1, const std::vector<cv::Point2f> &points2,
                             const cv::Mat &essentialMat, std::vector<cv::Point2f> &_maskedPoints1,
                             std::vector<cv::Point2f> &_maskedPoints2, cv::Mat &_rotationMat,
                             cv::Mat &_translationVec) {
    std::vector<uchar> mask;
    // Parameters:
    //      E           The input essential matrix.
    //      points1     Array of N 2D points from the first image. The point coordinates should be floating-point (single or double precision).
    //      points2     Array of the second image points of the same size and format as points1 .
    //      R           Recovered relative rotation.
    //      t           Recoverd relative translation.
    //      focal       Focal length of the camera. Note that this function assumes that points1 and points2 are feature points from cameras with same focal length and principle point.
    //      pp          Principle point of the camera.
    //      mask        Input/output mask for inliers in points1 and points2. If it is not empty, then it marks inliers in points1 and points2 for then given essential matrix E. Only these inliers will be used to recover pose. In the output mask only inliers which pass the cheirality check.
    cv::recoverPose(essentialMat, points1, points2, _rotationMat, _translationVec, kFocalLength, kPrinciplePoint, mask);
    std::vector<cv::Point2f> maskedPoint1;
    std::vector<cv::Point2f> maskedPoint2;
    for (int i = 0; i < mask.size(); i++) {
        if (mask[i]) {
            maskedPoint1.push_back(points1[i]);
            maskedPoint2.push_back(points2[i]);
        }
    }
    _maskedPoints1 = maskedPoint1;
    _maskedPoints2 = maskedPoint2;
}

double CalcPitchAngle(const cv::Mat &rotationMat) {
    return asin(-rotationMat.at<double>(1, 2));
}


void EstimateMotion(const std::vector<cv::Point2f> &points1, const std::vector<cv::Point2f> &points2,
                    std::vector<cv::Point2f> &_maskedPoints1, std::vector<cv::Point2f> &_maskedPoints2,
                    double &_pitch) {
    std::vector<cv::Point2f> maskedPoints1;
    std::vector<cv::Point2f> maskedPoints2;
    cv::Mat f;
    CalcFundamentalMat(points1, points2, maskedPoints1, maskedPoints2, f);
    double paramK[] = {kFocalLength, 0, kPrinciplePoint.x,
                       0, kFocalLength, kPrinciplePoint.y,
                       0, 0, 1};
    cv::Mat k = cv::Mat(3, 3, CV_64FC1, paramK);
    cv::Mat e;
    CalcEssentialMat(f, k, e);
    cv::Mat r;
    cv::Mat t;

    CalcExtrinsicParameters(maskedPoints1, maskedPoints2, e, _maskedPoints1, _maskedPoints2, r, t);
    _pitch = CalcPitchAngle(r);

    std::cout << r << std::endl;
    std::cout << t << std::endl;
    std::cout << "ピッチ角:" << _pitch * 180 / M_PI << std::endl;
}


} // namespace pac