#include "utils.h"
#include <fstream>
#include <sstream>
#include <set>

// ----------------------------------------------------------------
// OBJ mesh: single projectPoints call covers all vertices per frame.
// Edges and faces store per-element colors from OBJ 'g' group names.
// ----------------------------------------------------------------
struct OBJMesh {
    std::vector<cv::Vec3f> verts;
    struct Edge { int a, b; cv::Scalar color; };
    struct Face { std::vector<int> idx; cv::Scalar color; };
    std::vector<Edge> edges;
    std::vector<Face> faces;
};

// Colors keyed by OBJ group name.
// Each entry: {edge_color, face_color}.
// NFS Most Wanted 2005 M3 GTR livery: silver body, M-blue wing, gold BBS wheels.
// Colors stored in BGR order (OpenCV convention).
static const std::map<std::string, std::pair<cv::Scalar,cv::Scalar>> GROUP_COLORS {
    {"body",    {cv::Scalar(215,215,220), cv::Scalar(28, 28, 35)}},  // BMW silver
    {"wing",    {cv::Scalar(200, 80, 10), cv::Scalar(60, 20,  5)}},  // M-sport blue
    {"wheels",  {cv::Scalar( 40,165,210), cv::Scalar(15, 60, 80)}},  // gold BBS
    {"default", {cv::Scalar(200,200,200), cv::Scalar(30, 30, 45)}},
};

// Load a Wavefront OBJ file.
// Transforms from car-local coords (X=front, Y=left, Z=up) to board coords.
// 'g' group directives in the file control per-part colors.
OBJMesh loadOBJ(const std::string& path, cv::Vec3f translate, float scale) {
    OBJMesh mesh;

    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: could not open " << path << "\n";
        return mesh;
    }

    std::vector<cv::Vec3f> raw_verts;
    std::set<std::pair<int,int>> edge_set;
    std::string line;
    std::string cur_group = "default";

    auto colors = [&]() -> const std::pair<cv::Scalar,cv::Scalar>& {
        auto it = GROUP_COLORS.find(cur_group);
        return it != GROUP_COLORS.end() ? it->second : GROUP_COLORS.at("default");
    };

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        std::string tok;
        iss >> tok;

        if (tok == "v") {
            float cx, cy, cz;
            iss >> cx >> cy >> cz;
            // Car-local → board: car-X→board-X, car-Y(left)→board-Y, car-Z→board-Z
            raw_verts.push_back({cx * scale + translate[0],
                                 cy * scale + translate[1],
                                 cz * scale + translate[2]});
        }
        else if (tok == "g") {
            iss >> cur_group;
        }
        else if (tok == "f") {
            std::vector<int> idx;
            std::string vtx;
            while (iss >> vtx) {
                int vi = std::stoi(vtx.substr(0, vtx.find('/')));
                vi = (vi < 0) ? (int)raw_verts.size() + vi : vi - 1;
                idx.push_back(vi);
            }
            if (idx.size() < 2) continue;

            auto& [ec, fc] = colors();

            // Register unique edges (first group that claims an edge wins its color)
            for (size_t i = 0; i < idx.size(); i++) {
                int a = idx[i], b = idx[(i + 1) % idx.size()];
                auto e = (a < b) ? std::make_pair(a,b) : std::make_pair(b,a);
                if (edge_set.insert(e).second)
                    mesh.edges.push_back({a, b, ec});
            }
            mesh.faces.push_back({idx, fc});
        }
    }

    mesh.verts = raw_verts;
    return mesh;
}

// Per-frame: detect checkerboard → solvePnP → project the entire mesh
// with a single projectPoints call, then fill faces and draw edges.
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

    // Axis endpoints: origin, +X, +Y (negative), +Z toward camera
    std::vector<cv::Vec3f> axis_points  = {{0,0,0},{3,0,0},{0,-3,0},{0,0,3}};
    std::vector<cv::Vec3f> board_corners = {{0,0,0},{8,0,0},{8,-5,0},{0,-5,0}};

    // Load E46 M3 GTR mesh.
    // Car local origin maps to board position (4, -2.5).
    // Car Y (left) maps to board Y because board Y increases toward top of board,
    // which is the left side when the car faces +X.
    OBJMesh car = loadOBJ(
        "car.obj",
        cv::Vec3f(4.0f, -2.5f, 0.0f),   // car center at board midpoint
        1.0f                              // 1 car-unit = 1 board-unit
    );

    if (car.verts.empty()) {
        std::cerr << "Error: car.obj could not be loaded or is empty\n";
        return 1;
    }
    std::cout << "Loaded car.obj: " << car.verts.size() << " verts, "
              << car.edges.size() << " edges, "
              << car.faces.size() << " faces\n";

    cv::Mat frame, gray, rvec, tvec;
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

            if (frame_count % 15 == 0)
                std::cout << "rvec: " << rvec.t() << "\ntvec: " << tvec.t() << "\n";

            // Draw axes: X=red, Y=green, Z=blue
            std::vector<cv::Point2f> axis_img, corners_img;
            cv::projectPoints(axis_points,   rvec, tvec, camera_matrix, dist_coeffs, axis_img);
            cv::projectPoints(board_corners, rvec, tvec, camera_matrix, dist_coeffs, corners_img);
            cv::line(frame, axis_img[0], axis_img[1], cv::Scalar(0,   0, 255), 3);
            cv::line(frame, axis_img[0], axis_img[2], cv::Scalar(0, 255,   0), 3);
            cv::line(frame, axis_img[0], axis_img[3], cv::Scalar(255,  0,   0), 3);
            for (const auto& pt : corners_img)
                cv::circle(frame, pt, 8, cv::Scalar(0, 255, 255), 2);

            // Project entire mesh with one call (efficient)
            std::vector<cv::Point2f> proj;
            cv::projectPoints(car.verts, rvec, tvec, camera_matrix, dist_coeffs, proj);

            // Translucent face fill: draw onto overlay, blend at 35% opacity
            cv::Mat overlay = frame.clone();
            for (const auto& face : car.faces) {
                std::vector<std::vector<cv::Point>> poly(1);
                for (int idx : face.idx)
                    poly[0].push_back(cv::Point(proj[idx]));
                cv::fillPoly(overlay, poly, face.color);
            }
            cv::addWeighted(overlay, 0.35, frame, 0.65, 0, frame);

            // Draw solid wireframe edges on top (per-group colors)
            for (const auto& e : car.edges)
                cv::line(frame, proj[e.a], proj[e.b], e.color, 1);
        }

        frame_count++;
        cv::imshow("Augmented Reality", frame);
        if (cv::waitKey(10) == 'q') break;
    }

    return 0;
}
