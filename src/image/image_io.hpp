#ifndef PITCHANGLECORRECTION_IMAGE_IO_H
#define PITCHANGLECORRECTION_IMAGE_IO_H

#include <iostream>
#include <vector>
#include <sys/stat.h>
#include <dirent.h>
#include <opencv2/opencv.hpp>


namespace pac {

void SearchDir(std::string DirPath, std::vector<std::string> &_filePaths);

cv::Mat readColorImage(const std::string &filePath);

cv::Mat readGrayImage(const std::string &filePath);

void showImage(const cv::Mat &image);

} // namespace pac

#endif //PITCHANGLECORRECTION_IMAGE_IO_H
