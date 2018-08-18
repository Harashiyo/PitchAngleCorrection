#include "optical_flow.hpp"

namespace pac {

const float kMinFlowLength = 1;
const float kMaxFlowLength = 35;

void CalcOpticalFlow(const cv::Mat &prevImage, const cv::Mat &currImage,
                     const std::vector<cv::Point2f> &prevFeatures, std::vector<cv::Point2f> &_currFeatures,
                     std::vector<uchar> &_featuresFound) {
    cv::Mat prevImageGray;
    if (prevImage.channels() == 1) {
        prevImageGray = prevImage.clone();
    } else {
        cv::cvtColor(prevImage, prevImageGray, cv::COLOR_BGR2GRAY);
    }
    cv::Mat currImageGray;
    if (currImage.channels() == 1) {
        currImageGray = currImage.clone();
    } else {
        cv::cvtColor(currImage, currImageGray, cv::COLOR_BGR2GRAY);
    }
    std::vector<float> featuresErrors;
    const cv::Size winSize = cv::Size(21, 21);
    const int maxLevel = 3;
    const cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
    const int flags = 0;
    const double minEigThreshold = 1e-4;
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
            prevImageGray,
            currImageGray,
            prevFeatures,
            _currFeatures,
            _featuresFound,
            featuresErrors,
            winSize,
            maxLevel,
            criteria,
            flags,
            minEigThreshold);
}

void
CalcOpticalFlowTwoFrames(const cv::Mat &prevImage, const cv::Mat &currImage, std::vector<cv::Point2f> &_prevFeatures,
                         std::vector<cv::Point2f> &_currFeatures) {
    std::vector<cv::Point2f> prevFeatures;
    DetectFeatures(prevImage, prevFeatures);
    std::vector<cv::Point2f> currFeatures;
    std::vector<uchar> featuresFound;
    CalcOpticalFlow(prevImage, currImage, prevFeatures, currFeatures, featuresFound);

    std::vector<cv::Point2f> prevFeaturesFound;
    std::vector<cv::Point2f> currFeaturesFound;
    for (int i = 0; i < featuresFound.size(); i++) {
        if (!featuresFound[i]) {
            continue;
        }
        if (cv::norm(prevFeatures[i] - currFeatures[i]) > kMaxFlowLength) {
            featuresFound[i] = 0;
            continue;
        }
        if (cv::norm(prevFeatures[i] - currFeatures[i]) < kMinFlowLength) {
            featuresFound[i] = 0;
            continue;
        }
        prevFeaturesFound.push_back(prevFeatures[i]);
        currFeaturesFound.push_back(currFeatures[i]);
    }
    _prevFeatures = prevFeaturesFound;
    _currFeatures = currFeaturesFound;
    return;
}

void CalcOpticalFlowMultFrames(const std::deque<cv::Mat> &images, std::vector<cv::Point2f> &_prevFeaturesFound,
                               std::vector<cv::Point2f> &_currFeaturesFound) {
    if (images.size() < 2) {
        fprintf(stderr, "error: more than 2 images are required\n");
        exit(1);
    }
    std::vector<cv::Point2f> initialFeatures;
    DetectFeatures(images.front(), initialFeatures);
    const int size = initialFeatures.size();
    uchar initialFlags[size];
    for (int i = 0; i < size; i++) {
        initialFlags[i] = 1;
    }
    std::vector<cv::Point2f> prevFeatures;
    std::copy(initialFeatures.begin(), initialFeatures.end(), std::back_inserter(prevFeatures));
    std::vector<cv::Point2f> currFeatures;
    std::vector<uchar> foundFlags;
    for (int i = 0; i < images.size() - 1; i++) {
        CalcOpticalFlow(images[i], images[i + 1], prevFeatures, currFeatures, foundFlags);
        std::vector<cv::Point2f> currFeaturesFound;
        int k = 0;
        for (int j = 0; j < size; j++) {
            if (initialFlags[j]) {
                if (foundFlags[k] && cv::norm(prevFeatures[k] - currFeatures[k]) <= kMaxFlowLength &&
                    cv::norm(prevFeatures[k] - currFeatures[k]) >= kMinFlowLength) {
                    currFeaturesFound.push_back(currFeatures[k]);
                } else {
                    initialFlags[j] = 0;
                }
                k++;
            }
        }
        prevFeatures = currFeaturesFound;
    }
    std::vector<cv::Point2f> prevFeaturesFound;
    for (int i = 0; i < size; i++) {
        if (initialFlags[i]) {
            prevFeaturesFound.push_back(initialFeatures[i]);
        }
    }
    _prevFeaturesFound = prevFeaturesFound;
    _currFeaturesFound = prevFeatures;
}

void DrawOpticalFlow(const cv::Mat &image, const std::vector<cv::Point2f> &prevFeatures,
                     const std::vector<cv::Point2f> &currFeatures, LineType l, cv::Mat &_result, int thickness,
                     const cv::Scalar &color) {
    _result = image.clone();
    switch (l) {
        case LINE_SEGMENT:
            for (int i = 0; i < prevFeatures.size(); i++) {
                cv::line(_result, prevFeatures[i], currFeatures[i], color, thickness);
            }
            break;
        case STRAIGHT_LINE:
            // 直線 : ax+by+c=0
            std::vector<cv::Vec3f> lines;
            CalcLines(prevFeatures,currFeatures,lines);
            DrawLines(image, lines, _result, thickness, color);
            break;
    }
    return;
}

void Normalization(const cv::Mat &image, const std::vector<cv::Point2f> &prevFeatures,
                   const std::vector<cv::Point2f> &currFeatures, std::vector<cv::Point2f> &_prevNormalized,
                   std::vector<cv::Point2f> &_currNormalized) {
    std::vector<cv::Point2f> prevNormalized;
    std::vector<cv::Point2f> currNormalized;
    float x = (float) image.cols / 2;
    float y = (float) image.rows / 2;
    for (int i = 0; i < prevFeatures.size(); i++) {
        prevNormalized.push_back(cv::Point2f(prevFeatures[i].x - x, prevFeatures[i].y - y));
        currNormalized.push_back(cv::Point2f(currFeatures[i].x - x, currFeatures[i].y - y));
    }
    _prevNormalized = prevNormalized;
    _currNormalized = currNormalized;
    return;
}

void Normalization2(const cv::Mat &image, const std::vector<cv::Point2f> &prevFeatures,
                    const std::vector<cv::Point2f> &currFeatures, std::vector<cv::Point2f> &_prevNormalized,
                    std::vector<cv::Point2f> &_currNormalized) {
    std::vector<cv::Point2f> prevNormalized;
    std::vector<cv::Point2f> currNormalized;
    const float focalLength = 1280;
    float t[] = {0.0, 0.0};
    for (int i = 0; i < prevFeatures.size(); i++) {
        prevNormalized.push_back(cv::Point2f(prevFeatures[i].x / focalLength, prevFeatures[i].y / focalLength));
        currNormalized.push_back(cv::Point2f(currFeatures[i].x / focalLength, currFeatures[i].y / focalLength));
        t[0] += prevNormalized[i].x + currNormalized[i].x;
        t[1] += prevNormalized[i].y + currNormalized[i].y;
    }
    t[0] /= prevFeatures.size() * 2;
    t[1] /= prevFeatures.size() * 2;
    std::vector<float> dist;
    for (int i = 0; i < prevFeatures.size(); i++) {
        prevNormalized[i].x = prevFeatures[i].x - t[0];
        prevNormalized[i].y = prevFeatures[i].y - t[1];
        currNormalized[i].x = currFeatures[i].x - t[0];
        currNormalized[i].y = currFeatures[i].y - t[1];
        dist.push_back(std::sqrt(std::pow(prevNormalized[i].x, 2) + std::pow(prevNormalized[i].y, 2)));
        dist.push_back(std::sqrt(std::pow(currNormalized[i].x, 2) + std::pow(currNormalized[i].y, 2)));
    }
    float meanDist = 0.0;
    for (float d:dist) {
        meanDist += d;
    }
    meanDist /= dist.size();
    float scale = (float) M_SQRT2 / meanDist;
    for (int i = 0; i < prevFeatures.size(); i++) {
        prevNormalized[i].x = prevFeatures[i].x * scale - scale * t[0] * focalLength;
        prevNormalized[i].y = prevFeatures[i].y * scale - scale * t[1] * focalLength;
        currNormalized[i].x = currFeatures[i].x * scale - scale * t[0] * focalLength;
        currNormalized[i].y = currFeatures[i].y * scale - scale * t[1] * focalLength;
    }
    _prevNormalized = prevNormalized;
    _currNormalized = currNormalized;
    return;
}


void LineFilter(const std::vector<cv::Vec3f> &lines, const cv::Point2f &upperLeft, const cv::Point2f &bottomRight,
                std::vector<cv::Vec3f> &_result) {
    std::vector<cv::Vec3f> result;
    for (cv::Vec3f l :lines) {
        if (l[1]) {
            float left = SolveY(l, upperLeft.x);
            if (left >= upperLeft.y && left < bottomRight.y) {
                result.push_back(l);
                continue;
            }
            float right = SolveY(l, bottomRight.x);
            if (right >= upperLeft.y && right < bottomRight.y) {
                result.push_back(l);
                continue;
            }
        }
        if (l[0]) {
            float top = SolveX(l, upperLeft.y);
            if (top >= upperLeft.x && top < bottomRight.x) {
                result.push_back(l);
                continue;
            }
            float bottom = SolveX(l, bottomRight.y);
            if (bottom >= upperLeft.x && bottom < bottomRight.x) {
                result.push_back(l);
            }
        }
    }
    _result = result;
    return;
}

void CalcFocusOfExpansion(const cv::Mat &image, const std::vector<cv::Point2f> &prevFeatures,
                          const std::vector<cv::Point2f> &currFeatures, cv::Point2f &_eof) {
    std::vector<cv::Vec3f> lines;
    CalcLines(prevFeatures,currFeatures,lines);
    float x = (float) image.cols / 5;
    float y = (float) image.rows / 3;
    float xx = x / 9;
    float yy = y / 9;
    float min[3] = {1000000, 0, 0};
    std::vector<cv::Vec3f> linesFiltrated;
    LineFilter(lines, cv::Point2f(x * 2, y), cv::Point2f(x * 3, y * 2), linesFiltrated);

    const int num = lines.size();
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            float sum = 0.0;
            for (cv::Vec3f l:lines) {
                sum += std::sqrt(CalcDistance(l, cv::Point2f(x * 2 + xx * i, y + yy * j)));
            }
            sum /= num;
            if (min[0] > sum) {
                min[0] = sum;
                min[1] = x * 2 + xx * i;
                min[2] = y + yy * j;
            }

        }
    }
    x = min[1];
    y = min[2];
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            float sum = 0.0;
            for (cv::Vec3f l:lines) {
                sum += std::sqrt(CalcDistance(l, cv::Point2f(x + i - 5, y + j - 5)));
            }
            sum /= num;
            if (min[0] > sum) {
                min[0] = sum;
                min[1] = x + i - 5;
                min[2] = y + j - 5;
            }
        }
    }
    _eof = cv::Point2f(min[1], min[2]);
    return;
}

} // namespace pac
