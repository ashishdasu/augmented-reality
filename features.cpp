#include "utils.h"

// Harris corner detector on live video.
// Trackbars adjust the detection threshold and neighborhood block size.
// Detected corners are drawn as small circles on the output frame.

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera\n";
        return 1;
    }

    cv::namedWindow("Features");
    // Pass nullptr and read with getTrackbarPos each frame (avoids deprecated pointer API)
    cv::createTrackbar("Threshold", "Features", nullptr, 255);
    cv::createTrackbar("Block size", "Features", nullptr, 10);
    cv::setTrackbarPos("Threshold",  "Features", 150);
    cv::setTrackbarPos("Block size", "Features", 2);

    cv::Mat frame, gray, response, response_norm;

    for (;;) {
        cap >> frame;
        if (frame.empty()) continue;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        int threshold_val = cv::getTrackbarPos("Threshold",  "Features");
        int block_size    = cv::getTrackbarPos("Block size", "Features");

        // Harris response: each pixel gets a score measuring corner-ness.
        // blockSize: local neighborhood; ksize: Sobel aperture; k: Harris free parameter.
        int bs = std::max(block_size, 2);   // block size must be >= 2
        cv::cornerHarris(gray, response, bs, 3, 0.04);

        // Normalize to [0, 255] so the trackbar threshold is intuitive
        cv::normalize(response, response_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1);

        // Mark pixels whose response exceeds the threshold
        for (int r = 0; r < response_norm.rows; r++) {
            for (int c = 0; c < response_norm.cols; c++) {
                if (response_norm.at<float>(r, c) > threshold_val)
                    cv::circle(frame, cv::Point(c, r), 4, cv::Scalar(0, 0, 255), 1);
            }
        }

        cv::imshow("Features", frame);
        if (cv::waitKey(10) == 'q') break;
    }

    return 0;
}
