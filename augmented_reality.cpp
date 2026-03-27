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

    // Axis tips: origin, +X, +Y (negative), +Z toward camera
    std::vector<cv::Vec3f> axis_points = {
        {0, 0, 0}, {3, 0, 0}, {0, -3, 0}, {0, 0, 3}
    };
    // Outer board corners for alignment verification
    std::vector<cv::Vec3f> board_corners = {
        {0, 0, 0}, {8, 0, 0}, {8, -5, 0}, {0, -5, 0}
    };

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

            if (frame_count % 15 == 0) {
                std::cout << "rvec: " << rvec.t() << "\n"
                          << "tvec: " << tvec.t() << "\n";
            }

            std::vector<cv::Point2f> axis_img, corners_img;
            cv::projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs, axis_img);
            cv::projectPoints(board_corners, rvec, tvec, camera_matrix, dist_coeffs, corners_img);

            cv::line(frame, axis_img[0], axis_img[1], cv::Scalar(0, 0, 255), 3);  // X = red
            cv::line(frame, axis_img[0], axis_img[2], cv::Scalar(0, 255, 0), 3);  // Y = green
            cv::line(frame, axis_img[0], axis_img[3], cv::Scalar(255, 0, 0), 3);  // Z = blue

            for (const auto& pt : corners_img)
                cv::circle(frame, pt, 8, cv::Scalar(0, 255, 255), 2);
        }

        frame_count++;
        cv::imshow("Augmented Reality", frame);
        if (cv::waitKey(10) == 'q') break;
    }

    return 0;
}
