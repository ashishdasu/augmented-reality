#include "utils.h"

int main(int argc, char* argv[]) {
    cv::Mat camera_matrix, dist_coeffs;
    if (!loadCalibration("calibration.yml", camera_matrix, dist_coeffs)) {
        std::cerr << "Error: could not load calibration.yml\n";
        return 1;
    }

    int cam_index = (argc > 1) ? std::stoi(argv[1]) : 0;
    cv::VideoCapture cap(cam_index);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera " << cam_index << "\n";
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    const cv::Size board_size(9, 6);
    std::vector<cv::Vec3f> point_set = generateWorldPoints();

    cv::Mat frame, gray;
    cv::Mat rvec, tvec;
    int frame_count = 0;

    for (;;) {
        cap >> frame;
        if (frame.empty()) continue;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corner_set;
        bool found = cv::findChessboardCorners(gray, board_size, corner_set,
            cv::CALIB_CB_ADAPTIVE_THRESH |
            cv::CALIB_CB_NORMALIZE_IMAGE |
            cv::CALIB_CB_FAST_CHECK);

        if (found) {
            cv::cornerSubPix(gray, corner_set, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001));

            cv::solvePnP(point_set, corner_set, camera_matrix, dist_coeffs, rvec, tvec);

            // Print pose every 15 frames to avoid spam
            if (frame_count % 15 == 0) {
                std::cout << "rvec: " << rvec.t() << "\n"
                          << "tvec: " << tvec.t() << "\n";
            }
        }

        frame_count++;
        cv::imshow("Augmented Reality", frame);
        if (cv::waitKey(10) == 'q') break;
    }

    return 0;
}
