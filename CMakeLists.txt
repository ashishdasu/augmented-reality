cmake_minimum_required(VERSION 3.14)
project(project4)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate ${OpenCV_LIBS})

add_executable(augmented_reality augmented_reality.cpp)
target_link_libraries(augmented_reality ${OpenCV_LIBS})

add_executable(features features.cpp)
target_link_libraries(features ${OpenCV_LIBS})
