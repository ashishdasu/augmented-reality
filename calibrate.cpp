#include "utils.h"

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera\n";
        return 1;
    }

    // Checkerboard dimensions: 9x6 internal corners (not squares)
    const cv::Size board_size(9, 6);

    cv::Mat frame, gray;
    std::vector<cv::Point2f> corner_set;
    std::vector<cv::Point2f> last_corners;
    bool last_found = false;

    // Accumulate corresponding 2D/3D point sets across calibration frames
    std::vector<std::vector<cv::Point2f>> corner_list;
    std::vector<std::vector<cv::Vec3f>>   point_list;

    for (;;) {
        cap >> frame;
        if (frame.empty()) continue;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        bool found = cv::findChessboardCorners(gray, board_size, corner_set,
            cv::CALIB_CB_ADAPTIVE_THRESH |
            cv::CALIB_CB_NORMALIZE_IMAGE |
            cv::CALIB_CB_FAST_CHECK);

        if (found) {
            cv::cornerSubPix(gray, corner_set, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001));

            cv::drawChessboardCorners(frame, board_size, corner_set, found);

            std::cout << "Corners: " << corner_set.size()
                      << "  first: " << corner_set[0] << "\n";
        }

        last_corners = corner_set;
        last_found   = found;

        // Show saved frame count on the live feed
        cv::putText(frame,
            "Saved: " + std::to_string(corner_list.size()),
            cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
            cv::Scalar(0, 255, 0), 2);

        cv::imshow("Calibrate", frame);

        int key = cv::waitKey(10);
        if (key == 'q') break;

        // Save current frame's corners and matching world points
        if (key == 's' && last_found) {
            corner_list.push_back(last_corners);
            point_list.push_back(generateWorldPoints());

            std::string filename = "cal_frame_" +
                std::to_string(corner_list.size()) + ".png";
            cv::imwrite(filename, frame);

            std::cout << "Saved calibration frame " << corner_list.size() << "\n";
        }
    }

    return 0;
}
