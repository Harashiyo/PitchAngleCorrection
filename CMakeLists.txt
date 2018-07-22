cmake_minimum_required(VERSION 3.11)
project(PitchAngleCorrection)

set(CMAKE_CXX_STANDARD 11)

add_executable(PitchAngleCorrection src/main.cpp)

find_package(OpenCV 3.4 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(PitchAngleCorrection ${OpenCV_LIBS})