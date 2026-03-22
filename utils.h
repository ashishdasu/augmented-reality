#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>

// Generate the 54 world-space coordinates for a 9x6 checkerboard.
// Origin at upper-left internal corner, +X right, Y decreasing downward,
// Z=0 (the board is planar). Units are square-lengths (1 unit = 1 square).
// Row-major order to match findChessboardCorners output.
inline std::vector<cv::Vec3f> generateWorldPoints(int cols = 9, int rows = 6) {
    std::vector<cv::Vec3f> points;
    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
            points.push_back(cv::Vec3f(c, -r, 0));
    return points;
}

// Load camera intrinsics saved by calibrate. Returns false if file not found.
inline bool loadCalibration(const std::string& filename,
                            cv::Mat& camera_matrix,
                            cv::Mat& dist_coeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"]   >> dist_coeffs;
    fs.release();
    return true;
}

#endif // UTILS_H
