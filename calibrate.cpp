#include "utils.h"

// Calibration pipeline:
//   's' -- save current frame to corner_list/point_list and write cal_frame_N.png
//   'c' -- run calibrateCamera (requires >= 5 saved frames), auto-saves calibration.yml
//   'w' -- manually save calibration.yml
//   'q' -- quit (auto-saves if calibration has been run)
int main(int argc, char* argv[]) {
    // Optional argument: camera index (default 0). Use 1 if iPhone Continuity
    // Camera takes index 0 and pushes the built-in webcam to index 1.
    int cam_index = (argc > 1) ? std::stoi(argv[1]) : 0;

    cv::VideoCapture cap(cam_index);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera " << cam_index << "\n";
        return 1;
    }
    // Request 1080p — OpenCV defaults to 640x480 regardless of camera capability
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    std::cout << "Using camera index " << cam_index << "  resolution: "
              << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << "\n";

    // Checkerboard dimensions: 9x6 internal corners (not squares)
    const cv::Size board_size(9, 6);

    cv::Mat frame, gray;
    cv::Mat camera_matrix, dist_coeffs;
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
            // Refine corner locations to sub-pixel accuracy for better calibration
            cv::cornerSubPix(gray, corner_set, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001));

            cv::drawChessboardCorners(frame, board_size, corner_set, found);
            std::cout << "Corners: " << corner_set.size()
                      << "  first: " << corner_set[0] << "\n";
        }

        last_corners = corner_set;
        last_found   = found;

        cv::putText(frame,
            "Saved: " + std::to_string(corner_list.size()),
            cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
            cv::Scalar(0, 255, 0), 2);

        cv::imshow("Calibrate", frame);

        int key = cv::waitKey(10);
        if (key == 'q') {
            if (!camera_matrix.empty()) {
                // Auto-save on quit if calibration was run but not yet saved
                cv::FileStorage fs("calibration.yml", cv::FileStorage::WRITE);
                fs << "camera_matrix" << camera_matrix;
                fs << "dist_coeffs"   << dist_coeffs;
                fs.release();
                std::cout << "Auto-saved calibration.yml on quit\n";
            }
            break;
        }

        if (key == 's' && last_found) {
            corner_list.push_back(last_corners);
            point_list.push_back(generateWorldPoints());

            std::string filename = "cal_frame_" +
                std::to_string(corner_list.size()) + ".png";
            cv::imwrite(filename, frame);
            std::cout << "Saved calibration frame " << corner_list.size() << "\n";
        }

        // Run calibration once at least 5 frames have been collected
        if (key == 'c' && corner_list.size() >= 5) {
            // Initialize camera matrix with principal point at image center
            camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
            camera_matrix.at<double>(0, 2) = frame.cols / 2.0;
            camera_matrix.at<double>(1, 2) = frame.rows / 2.0;

            dist_coeffs = cv::Mat::zeros(5, 1, CV_64FC1);

            std::vector<cv::Mat> rvecs, tvecs;

            std::cout << "Camera matrix (initial):\n" << camera_matrix << "\n";

            double error = cv::calibrateCamera(point_list, corner_list,
                frame.size(), camera_matrix, dist_coeffs,
                rvecs, tvecs, cv::CALIB_FIX_ASPECT_RATIO);

            std::cout << "Camera matrix:\n"  << camera_matrix  << "\n"
                      << "Dist coeffs:\n"    << dist_coeffs    << "\n"
                      << "Reprojection error: " << error       << "\n";

            // Auto-save immediately after every calibration run
            cv::FileStorage fs("calibration.yml", cv::FileStorage::WRITE);
            fs << "camera_matrix" << camera_matrix;
            fs << "dist_coeffs"   << dist_coeffs;
            fs.release();
            std::cout << "Auto-saved calibration.yml (error: " << error << ")\n";
        }

        // Save calibration to file
        if (key == 'w' && !camera_matrix.empty()) {
            cv::FileStorage fs("calibration.yml", cv::FileStorage::WRITE);
            fs << "camera_matrix" << camera_matrix;
            fs << "dist_coeffs"   << dist_coeffs;
            fs.release();
            std::cout << "Calibration saved to calibration.yml\n";
        }
    }

    return 0;
}
