#include "image/image_io.hpp"
#include "optical_flow/optical_flow.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace pitchanglecorrection::opticalflow;
using namespace pitchanglecorrection::image;

int main(int argc, char *argv[]) {
    if(argc < 1){
        cout << "usage: ./a.out [images directory path]" << endl;
    }

    vector<string> fileList;
    SearchDir(argv[1], fileList);
    Mat frame = imread(fileList[0], 1);
    if (!frame.data) {
        cout << "No image data" << endl;
        return -1;
    }

    OpticalFlow driveRecorder(frame);

    for (int i = 1; i < fileList.size(); i++) {

        frame = imread(fileList[i], 1);
        if (!frame.data) {
            cout << "No image data" << endl;
            return -1;
        }
        std::vector< std::vector<cv::Point2f>> features;
        driveRecorder.CalcOpticalFlow(frame,features);
        Mat result;
        driveRecorder.DrawOpticalFlow(frame,OpticalFlow::STRAIGHT_LINE,result);

        cvNamedWindow("Display Image", CV_WINDOW_AUTOSIZE);
        imshow("Display Image", result);
        cvWaitKey(0);

    }
    return 0;
}

