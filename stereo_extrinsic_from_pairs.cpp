#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static void printUsage() {
    std::cout
        << "stereo_extrinsic_from_pairs\n"
        << "\nUsage:\n"
        << "  stereo_extrinsic_from_pairs.exe --pairs-dir data/pairs_cam1_cam2 \\\n"
        << "      --intr1 data/calib_cam1/intrinsic.yml --intr2 data/calib_cam2/intrinsic.yml \\\n"
        << "      --pattern 8x5 --square-mm 65 --output data/pairs_cam1_cam2/stereo_extrinsic.yml\n"
    << "\nOptions:\n"
    << "  --show          Visualize detections while scanning pairs\n"
    << "  --pause-ms N    Wait N ms between displayed pairs (default 1, use 0 to pause on key)\n"
    << "  --auto-scale-intrinsics  If intrinsics contain image_width/image_height and pair images differ, scale fx/fy/cx/cy to match\n"
        << "\nNotes:\n"
        << "- Expects image pairs named cam1_####.png and cam2_####.png in --pairs-dir\n"
        << "- Uses stereoCalibrate with fixed intrinsics\n";
}

static bool parsePattern(const std::string& s, cv::Size& pattern) {
    const auto x = s.find('x');
    if (x == std::string::npos) return false;
    try {
        int w = std::stoi(s.substr(0, x));
        int h = std::stoi(s.substr(x + 1));
        if (w <= 0 || h <= 0) return false;
        pattern = cv::Size(w, h);
        return true;
    } catch (...) {
        return false;
    }
}

static bool detectChessboard(const cv::Mat& gray, const cv::Size& pattern, std::vector<cv::Point2f>& corners) {
    corners.clear();
    if (gray.empty()) return false;

    bool found = false;
#if (CV_VERSION_MAJOR >= 4)
    // First try the more robust SB detector; then fall back to classic.
    found = cv::findChessboardCornersSB(gray, pattern, corners, cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE);
    if (!found) {
        corners.clear();
        found = cv::findChessboardCorners(gray, pattern, corners,
                                          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    }
#else
    found = cv::findChessboardCorners(gray, pattern, corners,
                                      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
#endif

    if (!found) {
        corners.clear();
        return false;
    }

    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 50, 1e-12));
    return true;
}

static std::vector<fs::path> listPairs(const fs::path& dir, std::vector<fs::path>& cam2Out) {
    std::map<std::string, fs::path> cam1;
    std::map<std::string, fs::path> cam2;

    for (const auto& e : fs::directory_iterator(dir)) {
        if (!e.is_regular_file()) continue;
        fs::path p = e.path();
        if (p.extension().string() != ".png") continue;
        const std::string name = p.filename().string();
        if (name.rfind("cam1_", 0) == 0) {
            cam1[name.substr(5)] = p; // "####.png"
        } else if (name.rfind("cam2_", 0) == 0) {
            cam2[name.substr(5)] = p;
        }
    }

    std::vector<fs::path> cam1List;
    cam2Out.clear();

    for (const auto& kv : cam1) {
        auto it = cam2.find(kv.first);
        if (it != cam2.end()) {
            cam1List.push_back(kv.second);
            cam2Out.push_back(it->second);
        }
    }

    return cam1List;
}

static bool loadIntrinsics(const fs::path& yml, cv::Mat& K, cv::Mat& D) {
    cv::FileStorage fs(yml.string(), cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;
    fs.release();
    return !K.empty() && !D.empty();
}

struct IntrinsicsInfo {
    cv::Mat K;
    cv::Mat D;
    int imageWidth = 0;
    int imageHeight = 0;
};

static bool loadIntrinsicsInfo(const fs::path& yml, IntrinsicsInfo& out) {
    out = IntrinsicsInfo{};
    cv::FileStorage fs(yml.string(), cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    fs["camera_matrix"] >> out.K;
    fs["distortion_coefficients"] >> out.D;

    if (!fs["image_width"].empty()) fs["image_width"] >> out.imageWidth;
    if (!fs["image_height"].empty()) fs["image_height"] >> out.imageHeight;

    fs.release();
    return !out.K.empty() && !out.D.empty();
}

static bool maybeScaleIntrinsicsToImageSize(const IntrinsicsInfo& src, const cv::Size& dstSize, cv::Mat& Kscaled) {
    Kscaled = src.K.clone();
    if (src.imageWidth <= 0 || src.imageHeight <= 0) return false;

    // Exact match: nothing to do.
    if (src.imageWidth == dstSize.width && src.imageHeight == dstSize.height) return true;

    // Swapped dimensions usually means portrait/landscape mismatch. Don't auto-scale that.
    if (src.imageWidth == dstSize.height && src.imageHeight == dstSize.width) return false;

    const double sx = (double)dstSize.width / (double)src.imageWidth;
    const double sy = (double)dstSize.height / (double)src.imageHeight;
    if (!(sx > 0.0 && sy > 0.0)) return false;

    // Scale fx, fy, cx, cy.
    Kscaled.at<double>(0, 0) *= sx;
    Kscaled.at<double>(1, 1) *= sy;
    Kscaled.at<double>(0, 2) *= sx;
    Kscaled.at<double>(1, 2) *= sy;
    return true;
}

static int stereoFlagsForDistortion(const cv::Mat& D) {
    // Match OpenCV's distortion model interpretation to the number of coefficients.
    // 5: k1 k2 p1 p2 k3
    // 8: + k4 k5 k6 (rational)
    // 12: + s1..s4 (thin prism)
    // 14: + tauX tauY (tilted)
    const int n = D.rows * D.cols;
    int flags = 0;
    if (n >= 8) flags |= cv::CALIB_RATIONAL_MODEL;
    if (n >= 12) flags |= cv::CALIB_THIN_PRISM_MODEL;
    if (n >= 14) flags |= cv::CALIB_TILTED_MODEL;
    return flags;
}

static std::vector<cv::Point3f> makeObjectPoints(const cv::Size& pattern, double squareMm) {
    std::vector<cv::Point3f> obj;
    obj.reserve((size_t)pattern.width * (size_t)pattern.height);
    for (int y = 0; y < pattern.height; ++y) {
        for (int x = 0; x < pattern.width; ++x) {
            obj.emplace_back((float)(x * squareMm), (float)(y * squareMm), 0.0f);
        }
    }
    return obj;
}

int main(int argc, char** argv) {
    fs::path pairsDir = fs::path("data") / "pairs_cam1_cam2";
    fs::path intr1 = fs::path("data") / "calib_cam1" / "intrinsic.yml";
    fs::path intr2 = fs::path("data") / "calib_cam2" / "intrinsic.yml";
    fs::path output = pairsDir / "stereo_extrinsic.yml";

    cv::Size pattern(8, 5);
    double squareMm = 65.0;
    bool show = false;
    int pauseMs = 1;
    bool autoScaleIntrinsics = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                std::exit(2);
            }
            return argv[++i];
        };

        if (a == "--help" || a == "-h") {
            printUsage();
            return 0;
        } else if (a == "--pairs-dir") {
            pairsDir = need("--pairs-dir");
        } else if (a == "--intr1") {
            intr1 = need("--intr1");
        } else if (a == "--intr2") {
            intr2 = need("--intr2");
        } else if (a == "--output") {
            output = need("--output");
        } else if (a == "--pattern") {
            cv::Size p;
            if (!parsePattern(need("--pattern"), p)) {
                std::cerr << "Bad --pattern; expected WxH like 8x5\n";
                return 2;
            }
            pattern = p;
        } else if (a == "--square-mm") {
            squareMm = std::stod(need("--square-mm"));
        } else if (a == "--show") {
            show = true;
        } else if (a == "--pause-ms") {
            pauseMs = std::stoi(need("--pause-ms"));
        } else if (a == "--auto-scale-intrinsics") {
            autoScaleIntrinsics = true;
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            printUsage();
            return 2;
        }
    }

    if (pauseMs < 0) {
        std::cerr << "ERROR: --pause-ms must be >= 0\n";
        return 2;
    }

    if (!fs::exists(pairsDir) || !fs::is_directory(pairsDir)) {
        std::cerr << "ERROR: Missing pairs dir: " << pairsDir.string() << "\n";
        return 1;
    }

    IntrinsicsInfo i1;
    IntrinsicsInfo i2;
    if (!loadIntrinsicsInfo(intr1, i1)) {
        std::cerr << "ERROR: Cannot read intr1: " << intr1.string() << "\n";
        return 1;
    }
    if (!loadIntrinsicsInfo(intr2, i2)) {
        std::cerr << "ERROR: Cannot read intr2: " << intr2.string() << "\n";
        return 1;
    }

    cv::Mat K1 = i1.K.clone();
    cv::Mat D1 = i1.D.clone();
    cv::Mat K2 = i2.K.clone();
    cv::Mat D2 = i2.D.clone();

    std::vector<fs::path> cam2Files;
    const auto cam1Files = listPairs(pairsDir, cam2Files);
    if (cam1Files.empty() || cam2Files.empty() || cam1Files.size() != cam2Files.size()) {
        std::cerr << "ERROR: No matched pairs in " << pairsDir.string() << " (need cam1_####.png and cam2_####.png)\n";
        return 1;
    }

    const auto obj = makeObjectPoints(pattern, squareMm);

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints1;
    std::vector<std::vector<cv::Point2f>> imagePoints2;

    cv::Size imageSize;

    size_t foundCam1Only = 0;
    size_t foundCam2Only = 0;
    size_t foundBoth = 0;
    size_t readFailures = 0;

    if (show) {
        cv::namedWindow("pair_cam1", cv::WINDOW_NORMAL);
        cv::namedWindow("pair_cam2", cv::WINDOW_NORMAL);
    }

    for (size_t i = 0; i < cam1Files.size(); ++i) {
        cv::Mat im1 = cv::imread(cam1Files[i].string(), cv::IMREAD_COLOR);
        cv::Mat im2 = cv::imread(cam2Files[i].string(), cv::IMREAD_COLOR);
        if (im1.empty() || im2.empty()) {
            ++readFailures;
            continue;
        }

        cv::Mat g1, g2;
        cv::cvtColor(im1, g1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(im2, g2, cv::COLOR_BGR2GRAY);

        if (imageSize.empty()) {
            imageSize = g1.size();

            if (i1.imageWidth > 0 && i1.imageHeight > 0) {
                if (i1.imageWidth != imageSize.width || i1.imageHeight != imageSize.height) {
                    std::cerr << "WARNING: intr1 calibrated at " << i1.imageWidth << "x" << i1.imageHeight
                              << " but pairs are " << imageSize.width << "x" << imageSize.height << "\n";
                    if (i1.imageWidth == imageSize.height && i1.imageHeight == imageSize.width) {
                        std::cerr << "WARNING: Looks like portrait/landscape swap. Recalibrate intrinsics at the SAME orientation/resolution as pairs.\n";
                    } else if (autoScaleIntrinsics) {
                        cv::Mat scaled;
                        if (maybeScaleIntrinsicsToImageSize(i1, imageSize, scaled)) {
                            K1 = scaled;
                            std::cerr << "INFO: Auto-scaled intr1 K to pair image size\n";
                        }
                    }
                }
            }
            if (i2.imageWidth > 0 && i2.imageHeight > 0) {
                if (i2.imageWidth != imageSize.width || i2.imageHeight != imageSize.height) {
                    std::cerr << "WARNING: intr2 calibrated at " << i2.imageWidth << "x" << i2.imageHeight
                              << " but pairs are " << imageSize.width << "x" << imageSize.height << "\n";
                    if (i2.imageWidth == imageSize.height && i2.imageHeight == imageSize.width) {
                        std::cerr << "WARNING: Looks like portrait/landscape swap. Recalibrate intrinsics at the SAME orientation/resolution as pairs.\n";
                    } else if (autoScaleIntrinsics) {
                        cv::Mat scaled;
                        if (maybeScaleIntrinsicsToImageSize(i2, imageSize, scaled)) {
                            K2 = scaled;
                            std::cerr << "INFO: Auto-scaled intr2 K to pair image size\n";
                        }
                    }
               