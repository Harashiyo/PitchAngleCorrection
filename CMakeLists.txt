cmake_minimum_required(VERSION 3.11)
project(PitchAngleCorrection)

set(CMAKE_CXX_STANDARD 11)

add_executable(PitchAngleCorrection src/main.cpp
                                    src/optical_flow/feature_detection.cpp
                                    src/optical_flow/feature_detection.hpp
                                    src/optical_flow/optical_flow.cpp
                                    src/optical_flow/optical_flow.hpp
                                    src/image/image_io.cpp
                                    src/image/image_io.hpp
                                    src/geometry/motion_estimation.cpp
                                    src/geometry/motion_estimation.hpp
                                    src/geometry/geometry.cpp
                                    src/geometry/geometry.hpp
                                    src/image/camera.hpp)

find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(PitchAngleCorrection ${OpenCV_LIBS})