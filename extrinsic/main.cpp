#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    std::string cameraId = "cam0";
    cv::Size pattern(8, 5);
    double squareMm = 65.0;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                std::exit(2);
            }
            return argv[++i];
        };

        if (a == "--camera-id") {
            cameraId = need("--camera-id");
        } else if (a == "--pattern") {
            const std::string s = need("--pattern");
            const auto x = s.find('x');
            if (x == std::string::npos) {
                std::cerr << "Bad --pattern; expected WxH like 8x5\n";
                return 2;
            }
            int w = std::stoi(s.substr(0, x));
            int h = std::stoi(s.substr(x + 1));
            if (w <= 0 || h <= 0) {
                std::cerr << "Bad --pattern; expected WxH like 8x5\n";
                return 2;
            }
            pattern = cv::Size(w, h);
        } else if (a == "--square-mm") {
            squareMm = std::stod(need("--square-mm"));
        } else if (a == "--help" || a == "-h") {
            std::cout
                << "extrinsic\\n"
                << "Usage:\\n"
                << "  extrinsic.exe --camera-id cam0 [--pattern 8x5] [--square-mm 65]\\n"
                << "\\n"
                << "Reads images from: data/<camera_id>/img_*.png\\n"
                << "Reads intrinsics from: data/<camera_id>/intrinsic.yml\\n"
                << "Writes extrinsics to: data/<camera_id>/extrinsic.yml\\n";
            return 0;
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            return 2;
        }
    }

    const fs::path dir = fs::path("data") / cameraId;
    const fs::path intrPath = dir / "intrinsic.yml";
    const fs::path outPath = dir / "extrinsic.yml";

    cv::Mat K, D;
    {
        cv::FileStorage fs(intrPath.string(), cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "ERROR: Cannot open intrinsics: " << intrPath.string() << "\n";
            return 1;
        }
        fs["camera_matrix"] >> K;
        fs["distortion_coefficients"] >> D;
        fs.release();
    }

    if (K.empty() || D.empty() || K.rows != 3 || K.cols != 3) {
        std::cerr << "ERROR: Invalid intrinsics in " << intrPath.string() << "\n";
        return 1;
    }

    std::vector<fs::path> files;
    if (!fs::exists(dir) || !fs::is_directory(dir)) {
        std::cerr << "ERROR: Missing input directory: " << dir.string() << "\n";
        return 1;
    }

    for (const auto& e : fs::directory_iterator(dir)) {
        if (!e.is_regular_file()) continue;
        const auto p = e.path();
        if (p.extension().string() != ".png") continue;
        const auto name = p.filename().string();
        if (name.rfind("img_", 0) != 0) continue;
        files.push_back(p);
    }
    std::sort(files.begin(), files.end());

    if (files.empty()) {
        std::cerr << "ERROR: No images found in " << dir.string() << "\n";
        return 1;
    }

    std::vector<cv::Point3f> obj;
    obj.reserve((size_t)pattern.width * (size_t)pattern.height);
    for (int y = 0; y < pattern.height; ++y) {
        for (int x = 0; x < pattern.width; ++x) {
            obj.push_back(cv::Point3f((float)(x * squareMm), (float)(y * squareMm), 0.0f));
        }
    }

    cv::Mat rvec, tvec;
    std::string usedImage;

    for (const auto& p : files) {
        cv::Mat img = cv::imread(p.string(), cv::IMREAD_COLOR);
        if (img.empty()) continue;

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            gray,
            pattern,
            corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (!found) continue;

        cv::cornerSubPix(
            gray,
            corners,
            cv::Size(11, 11),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 50, 0.0001));

        bool ok = cv::solvePnP(obj, corners, K, D, rvec, tvec);
        if (!ok) continue;

        usedImage = p.filename().string();
        break;
    }

    if (usedImage.empty() || rvec.empty() || tvec.empty()) {
        std::cerr << "ERROR: Could not estimate pose from any image.\n";
        return 1;
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);

    cv::FileStorage out(outPath.string(), cv::FileStorage::WRITE);
    if (!out.isOpened()) {
        std::cerr << "ERROR: Cannot open for writing: " << outPath.string() << "\n";
        return 1;
    }

    out << "camera_id" << cameraId;
    out << "image_used" << usedImage;
    out << "pattern_size_w" << pattern.width;
    out << "pattern_size_h" << pattern.height;
    out << "square_size_mm" << squareMm;
    out << "rvec" << rvec;
    out << "tvec" << tvec;
    out << "R" << R;
    out.release();

    return 0;
}
