#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/stat.h>
#include <dirent.h>

using namespace cv;
using namespace std;

RNG rng(12345);

void search_dir(string path, vector<string> &fileNames);

void detectFeatures(Mat &image, vector<Point2f> &features);

int main(int argc, char* argv[]) {
    vector<string> fileList;
    search_dir(argv[1], fileList);
    sort(fileList.begin(), fileList.end());

    Mat prevFrame, prevFrameGray;
    prevFrame = imread(fileList[0], 1);
    if (!prevFrame.data) {
        printf("No image data\n");
        return -1;
    }
    cvtColor(prevFrame, prevFrameGray, COLOR_BGR2GRAY);
    vector<Point2f> prevCorners;

    detectFeatures(prevFrameGray, prevCorners);

    for (int j = 1; j < fileList.size(); j++) {
        Mat currFrame, currFrameGray;
        vector<Point2f> currCorners;
        currFrame = imread(fileList[j], 1);
        if (!currFrame.data) {
            printf("No image data\n");
            return -1;
        }
        cvtColor(currFrame, currFrameGray, COLOR_BGR2GRAY);

        detectFeatures(currFrameGray, currCorners);

        vector<uchar> featuresFound;
        vector<float> featuresErrors;

        calcOpticalFlowPyrLK(
                prevFrameGray,
                currFrameGray,
                prevCorners,
                currCorners,
                featuresFound,
                featuresErrors);

        Mat frame;
        frame = currFrame.clone();
        for (int i = 0; i < featuresFound.size(); i++) {
            Point p1 = Point((int) prevCorners[i].x, (int) prevCorners[i].y);
            Point p2 = Point((int) currCorners[i].x, (int) currCorners[i].y);

            float slope = (float) (p2.y - p1.y) / (p2.x - p1.x);
            float intercept = (float) p1.y - slope * p1.x;
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
            Point left = Point((int) x1, (int) y1);
            Point right = Point((int) x2, (int) y2);
            //line(frame, left, right, Scalar(0, 0, 255), 4);
            line(frame, p1, p2, Scalar(0, 0, 255), 12);

        }
        int r = 4;
        for (size_t i = 0; i < currCorners.size(); i++) {
            circle(frame, currCorners[i], r, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), -1,
                   24, 0);
        }
        for (size_t i = 0; i < prevCorners.size(); i++) {
            circle(frame, prevCorners[i], r, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), -1,
                   8, 0);
        }
        prevFrameGray = currFrameGray.clone();
        prevCorners.clear();
        copy(currCorners.begin(), currCorners.end(), back_inserter(prevCorners));

        cvNamedWindow("Display Image", CV_WINDOW_AUTOSIZE);
        imshow("Display Image", frame);
        cvWaitKey(0);
    }
    return 0;
}

void search_dir(string path, vector<string> &fileNames) {

    int i, dirElements;
    string search_path;

    struct stat stat_buf;
    struct dirent **namelist = NULL;

    // dirElements にはディレクトリ内の要素数が入る
    dirElements = scandir(path.c_str(), &namelist, NULL, NULL);

    if (dirElements == -1) {
        cout << "ERROR" << endl;
    } else {

        //ディレクトリかファイルかを順番に識別
        for (i = 0; i < dirElements; i += 1) {

            // "." と ".." を除く
            if ((strcmp(namelist[i]->d_name, ".\0") != 0) && (strcmp(namelist[i]->d_name, "..\0") != 0)) {

                //search_pathには検索対象のフルパスを格納する
                search_path = path + string(namelist[i]->d_name);

                // ファイル情報の取得の成功
                if (stat(search_path.c_str(), &stat_buf) == 0) {

                    // ディレクトリだった場合の処理
                    if ((stat_buf.st_mode & S_IFMT) == S_IFDIR) {
                        // 再帰によりディレクトリ内を探索
                        search_dir(path + string(namelist[i]->d_name) + "/", fileNames);
                    }

                        //ファイルだった場合の処理
                    else {
                        fileNames.push_back(search_path);
                    }
                }

                    // ファイル情報の取得の失敗
                else {
                    cout << "ERROR" << endl << endl;
                }
            }
        }
    }

    free(namelist);
    return;
}

void detectFeatures(Mat &grayImage, vector<Point2f> &features) {
    const int maxCorners = 100;
    const double qualityLevel = 0.05;
    const double minDistance = 10;
    const int blockSize = 3;
    const bool useHarrisDetector = true;
    const double k = 0.04;
    int splitHeight = grayImage.rows / 3;

    Rect roi_top(0, 0, grayImage.cols, splitHeight);
    Rect roi_middle(0, splitHeight, grayImage.cols, splitHeight);
    Rect roi_bottom(0, splitHeight * 2, grayImage.cols, splitHeight);
    Mat grayImage_top = grayImage(roi_top);
    Mat grayImage_middle = grayImage(roi_middle);
    Mat grayImage_bottom = grayImage(roi_bottom);
    vector<Point2f> features_top;
    vector<Point2f> features_middle;
    vector<Point2f> features_bottom;

    goodFeaturesToTrack(grayImage_top,
                        features_top,
                        maxCorners,
                        qualityLevel,
                        minDistance,
                        Mat(),
                        blockSize,
                        useHarrisDetector,
                        k);
    cornerSubPix(grayImage_top, features_top, Size(21, 21), Size(-1, -1),
                 TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 0.01));

    goodFeaturesToTrack(grayImage_middle,
                        features_middle,
                        maxCorners,
                        qualityLevel,
                        minDistance,
                        Mat(),
                        blockSize,
                        useHarrisDetector,
                        k);
    cornerSubPix(grayImage_middle, features_middle, Size(21, 21), Size(-1, -1),
                 TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 0.01));

    goodFeaturesToTrack(grayImage_bottom,
                        features_bottom,
                        maxCorners,
                        qualityLevel,
                        minDistance,
                        Mat(),
                        blockSize,
                        useHarrisDetector,
                        k);
    cornerSubPix(grayImage_bottom, features_bottom, Size(21, 21), Size(-1, -1),
                 TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 0.01));
    for (int i = 0; i < features_middle.size(); i++) {
        features_middle[i].y += splitHeight;
    }
    for (int i = 0; i < features_bottom.size(); i++) {
        features_bottom[i].y += splitHeight * 2;
    }
    features.insert(features.end(), features_top.begin(), features_top.end());
    features.insert(features.end(), features_middle.begin(), features_middle.end());
    features.insert(features.end(), features_bottom.begin(), features_bottom.end());
}