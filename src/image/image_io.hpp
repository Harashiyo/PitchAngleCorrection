#ifndef PITCHANGLECORRECTION_IMAGE_IO_H
#define PITCHANGLECORRECTION_IMAGE_IO_H

#include <iostream>
#include <vector>
#include <sys/stat.h>
#include <dirent.h>


namespace pitchanglecorrection {
namespace image {

void SearchDir(std::string path, std::vector<std::string> &output);

}
}

#endif //PITCHANGLECORRECTION_IMAGE_IO_H
