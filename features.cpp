#include "utils.h"

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera\n";
        return 1;
    }

    cv::Mat frame;
    for (;;) {
        cap >> frame;
        if (frame.empty()) continue;

        cv::imshow("Features", frame);
        if (cv::waitKey(10) == 'q') break;
    }

    return 0;
}
