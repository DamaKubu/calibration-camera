#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <Windows.h>
#endif

namespace fs = std::filesystem;

static void printUsage() {
    std::cout
        << "intrinsic (offline)\n"
        << "Usage:\n"
        << "  intrinsic.exe --camera-id calib_cam1 [--pattern 8x5] [--square-mm 65] [--model standard]\n"
        << "  intrinsic.exe --images-dir data/calib_cam1 [--pattern 8x5] [--square-mm 65] [--model fisheye]\n"
        << "\n"
        << "Options:\n"
        << "  --camera-id <name>              Uses images in data/<name>/\n"
        << "  --images-dir <dir>              Directory with calibration images (absolute or relative)\n"
        << "  --ext <png|jpg>                 Extension filter (default png)\n"
        << "  --pattern <WxH>                 Inner corners, e.g. 8x5 (default 8x5)\n"
        << "  --square-mm <float>             Square size in mm (default 65)\n"
        << "  --model <standard|fisheye>      Lens model (default standard)\n"
        << "  --min-detections <int>          Minimum valid detections required (default 10)\n"
        << "  --max-per-image-error <float>   Reject images above this reproj error (px) (default 1.2)\n"
        << "  --max-remove <int>              Max outliers to remove (default 15)\n"
        << "  --output <file.yml>             Output YAML file (default: <images-dir>/intrinsic.yml)\n";
}

static std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    return s;
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

static fs::path executablePath() {
#if defined(_WIN32)
    wchar_t buf[MAX_PATH];
    const DWORD len = GetModuleFileNameW(nullptr, buf, MAX_PATH);
    if (len == 0 || len >= MAX_PATH) return {};
    return fs::path(buf);
#else
    return {};
#endif
}

static fs::path findDataRoot() {
    // Prefer current working directory.
    const fs::path cwd = fs::current_path();
    if (fs::exists(cwd / "data") && fs::is_directory(cwd / "data")) return cwd;

    // If launched from build/Release, walk up from executable location.
    fs::path exe = executablePath();
    if (!exe.empty()) {
        fs::path dir = exe.parent_path();
        for (int i = 0; i < 6 && !dir.empty(); ++i) {
            if (fs::exists(dir / "data") && fs::is_directory(dir / "data")) return dir;
            dir = dir.parent_path();
        }
    }

    // Fallback.
    return cwd;
}

static std::vector<fs::path> listImages(const fs::path& dir, const std::string& extLower) {
    std::vector<fs::path> files;
    if (!fs::exists(dir) || !fs::is_directory(dir)) return files;

    for (const auto& e : fs::directory_iterator(dir)) {
        if (!e.is_regular_file()) continue;
        fs::path p = e.path();
        std::string ext = toLower(p.extension().string());
        if (ext != ("." + extLower)) continue;
        files.push_back(p);
    }
    std::sort(files.begin(), files.end());
    return files;
}

static double perImageReprojError(
    const std::vector<cv::Point3f>& obj,
    const std::vector<cv::Point2f>& img,
    const cv::Mat& rvec,
    const cv::Mat& tvec,
    const cv::Mat& K,
    const cv::Mat& D,
    bool fisheye) {
    std::vector<cv::Point2f> proj;
    if (fisheye) {
        cv::fisheye::projectPoints(obj, proj, rvec, tvec, K, D);
    } else {
        cv::projectPoints(obj, rvec, tvec, K, D, proj);
    }
    if (proj.size() != img.size() || img.empty()) return 1e9;

    double sum = 0.0;
    for (size_t i = 0; i < img.size(); ++i) {
        const cv::Point2f d = proj[i] - img[i];
        sum += d.x * d.x + d.y * d.y;
    }
    return std::sqrt(sum / static_cast<double>(img.size()));
}

static double mean(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

static void yamlWriteString(std::ostream& os, const std::string& key, const std::string& value) {
    os << key << ": \"";
    for (char c : value) {
        if (c == '\\' || c == '"') os << '\\';
        os << c;
    }
    os << "\"\n";
}

static void yamlWriteInt(std::ostream& os, const std::string& key, int value) {
    os << key << ": " << value << "\n";
}

static void yamlWriteDouble(std::ostream& os, const std::string& key, double value) {
    os.setf(std::ios::fixed);
    os.precision(10);
    os << key << ": " << value << "\n";
}

static void yamlWriteOpenCvMatrix(std::ostream& os, const std::string& key, const cv::Mat& m) {
    cv::Mat md;
    m.convertTo(md, CV_64F);

    os << key << ": !!opencv-matrix\n";
    os << "   rows: " << md.rows << "\n";
    os << "   cols: " << md.cols << "\n";
    os << "   dt: d\n";
    os << "   data: [ ";
    os.setf(std::ios::fixed);
    os.precision(15);
    for (int r = 0; r < md.rows; ++r) {
        for (int c = 0; c < md.cols; ++c) {
            const double v = md.at<double>(r, c);
            os << v;
            if (!(r == md.rows - 1 && c == md.cols - 1)) os << ", ";
        }
    }
    os << " ]\n";
}

int main(int argc, char** argv) {
    std::string cameraId;
    fs::path imagesDir;
    std::string extLower = "png";
    cv::Size pattern(8, 5);
    double squareMm = 65.0;
    std::string model = "standard";
    int minDetections = 10;
    double maxPerImageError = 1.2;
    int maxRemove = 15;
    fs::path output;

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
        } else if (a == "--camera-id") {
            cameraId = need("--camera-id");
        } else if (a == "--images-dir") {
            imagesDir = need("--images-dir");
        } else if (a == "--ext") {
            extLower = toLower(need("--ext"));
        } else if (a == "--pattern") {
            cv::Size p;
            if (!parsePattern(need("--pattern"), p)) {
                std::cerr << "Bad --pattern; expected WxH like 8x5\n";
                return 2;
            }
            pattern = p;
        } else if (a == "--square-mm") {
            squareMm = std::stod(need("--square-mm"));
        } else if (a == "--model") {
            model = toLower(need("--model"));
            if (model != "standard" && model != "fisheye") {
                std::cerr << "--model must be 'standard' or 'fisheye'\n";
                return 2;
            }
        } else if (a == "--min-detections") {
            minDetections = std::stoi(need("--min-detections"));
        } else if (a == "--max-per-image-error") {
            maxPerImageError = std::stod(need("--max-per-image-error"));
        } else if (a == "--max-remove") {
            maxRemove = std::stoi(need("--max-remove"));
        } else if (a == "--output") {
            output = need("--output");
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            printUsage();
            return 2;
        }
    }

    if (cameraId.empty() && imagesDir.empty()) {
        std::cerr << "ERROR: Provide --camera-id or --images-dir\n";
        printUsage();
        return 2;
    }

    const fs::path dataRoot = findDataRoot();
    if (imagesDir.empty()) {
        imagesDir = dataRoot / "data" / cameraId;
    } else if (imagesDir.is_relative()) {
        // Allow passing "calib_cam1" or "data/calib_cam1".
        if (fs::exists(dataRoot / imagesDir)) {
            imagesDir = dataRoot / imagesDir;
        } else {
            imagesDir = dataRoot / "data" / imagesDir;
        }
    }

    if (output.empty()) {
        output = imagesDir / "intrinsic.yml";
    } else if (output.is_relative()) {
        output = dataRoot / output;
    }

    if (!fs::exists(imagesDir) || !fs::is_directory(imagesDir)) {
        std::cerr << "ERROR: Missing images directory: " << imagesDir.string() << "\n";
        return 1;
    }

    const bool useFisheye = (model == "fisheye");
    const auto files = listImages(imagesDir, extLower);
    if (files.empty()) {
        std::cerr << "ERROR: No images found in " << imagesDir.string() << " with extension " << extLower << "\n";
        return 1;
    }

    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<fs::path> imagePaths;
    cv::Size imageSize;

    std::cout << "Build: " << __DATE__ << " " << __TIME__ << "\n";
    std::cout << "Images dir: " << imagesDir.string() << "\n";
    std::cout << "Found " << files.size() << " images (" << extLower << ")\n";
    std::cout << "Detecting chessboards...\n";

    for (const auto& p : files) {
        cv::Mat img = cv::imread(p.string(), cv::IMREAD_COLOR);
        if (img.empty()) continue;

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        if (imageSize.empty()) {
            imageSize = gray.size();
        } else if (gray.size() != imageSize) {
            std::cerr << "ERROR: Mixed image sizes. Found " << gray.cols << "x" << gray.rows
                      << " but expected " << imageSize.width << "x" << imageSize.height << "\n";
            return 1;
        }

        std::vector<cv::Point2f> corners;
        bool found = false;
#if (CV_VERSION_MAJOR >= 4)
        found = cv::findChessboardCornersSB(
            gray,
            pattern,
            corners,
            cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
#else
        found = cv::findChessboardCorners(
            gray,
            pattern,
            corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
#endif
        if (!found) continue;

        cv::cornerSubPix(
            gray,
            corners,
            cv::Size(15, 15),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 80, 1e-12));

        imagePoints.push_back(std::move(corners));
        imagePaths.push_back(p);
    }

    if ((int)imagePoints.size() < minDetections) {
        std::cerr << "ERROR: Not enough valid detections: " << imagePoints.size() << " (need >= " << minDetections << ")\n";
        return 1;
    }

    // Object points template
    std::vector<cv::Point3f> obj;
    obj.reserve((size_t)pattern.width * (size_t)pattern.height);
    for (int y = 0; y < pattern.height; ++y) {
        for (int x = 0; x < pattern.width; ++x) {
            obj.emplace_back((float)(x * squareMm), (float)(y * squareMm), 0.0f);
        }
    }

    std::vector<int> kept((int)imagePoints.size());
    std::iota(kept.begin(), kept.end(), 0);

    struct Rejected {
        fs::path path;
        double errorPx;
    };
    std::vector<Rejected> rejected;

    auto runCalib = [&](const std::vector<int>& idxs,
                        cv::Mat& K,
                        cv::Mat& D,
                        std::vector<cv::Mat>& rvecs,
                        std::vector<cv::Mat>& tvecs,
                        std::vector<double>& perErr) -> double {
        std::vector<std::vector<cv::Point3f>> objectPoints;
        std::vector<std::vector<cv::Point2f>> imgPts;
        objectPoints.reserve(idxs.size());
        imgPts.reserve(idxs.size());

        for (int i : idxs) {
            objectPoints.push_back(obj);
            imgPts.push_back(imagePoints[(size_t)i]);
        }

        if (useFisheye) {
            K = cv::Mat::zeros(3, 3, CV_64F);
            D = cv::Mat::zeros(4, 1, CV_64F);
        } else {
            K = cv::Mat::eye(3, 3, CV_64F);
            D = cv::Mat::zeros(8, 1, CV_64F); // k1..k6 + p1,p2
        }
        rvecs.clear();
        tvecs.clear();

        const cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 500, 1e-12);

        double rms = 0.0;
        if (useFisheye) {
            const int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
                              cv::fisheye::CALIB_CHECK_COND |
                              cv::fisheye::CALIB_FIX_SKEW;
            rms = cv::fisheye::calibrate(objectPoints, imgPts, imageSize, K, D, rvecs, tvecs, flags, criteria);
        } else {
            const int flags = cv::CALIB_RATIONAL_MODEL;
            rms = cv::calibrateCamera(objectPoints, imgPts, imageSize, K, D, rvecs, tvecs, flags, criteria);
        }

        perErr.clear();
        perErr.reserve(idxs.size());
        for (size_t j = 0; j < idxs.size(); ++j) {
            const int idx = idxs[j];
            perErr.push_back(perImageReprojError(obj, imagePoints[(size_t)idx], rvecs[j], tvecs[j], K, D, useFisheye));
        }

        return rms;
    };

    cv::Mat K, D;
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> perErr;
    double rms = 0.0;

    for (int iter = 0; iter <= maxRemove; ++iter) {
        rms = runCalib(kept, K, D, rvecs, tvecs, perErr);
        auto itWorst = std::max_element(perErr.begin(), perErr.end());
        const double worst = *itWorst;
        const int worstPos = (int)std::distance(perErr.begin(), itWorst);

        std::cout << "Iter " << iter << ": kept=" << kept.size()
                  << " rms=" << rms
                  << " mean=" << mean(perErr)
                  << " max=" << worst << "\n";

        if (worst > maxPerImageError && kept.size() > (size_t)minDetections) {
            const int worstIdx = kept[(size_t)worstPos];
            rejected.push_back({imagePaths[(size_t)worstIdx], worst});
            kept.erase(kept.begin() + worstPos);
            continue;
        }
        break;
    }

    const double maxErr = *std::max_element(perErr.begin(), perErr.end());

    std::cout << "\n=== FINAL INTRINSICS ===\n";
    std::cout << "Model:        " << model << "\n";
    std::cout << "Used images:  " << kept.size() << " (rejected " << rejected.size() << ")\n";
    std::cout << "RMS (solver): " << rms << " px\n";
    std::cout << "Mean reproj:  " << mean(perErr) << " px\n";
    std::cout << "Max reproj:   " << maxErr << " px\n";
    std::cout << "Image size:   " << imageSize.width << "x" << imageSize.height << "\n";

    fs::create_directories(output.parent_path());
    std::ofstream out(output);
    if (!out.is_open()) {
        std::cerr << "ERROR: Cannot write " << fs::absolute(output).string() << "\n";
        return 1;
    }

    out << "%YAML:1.0\n---\n";
    yamlWriteString(out, "build", std::string(__DATE__) + " " + __TIME__);
    if (!cameraId.empty()) yamlWriteString(out, "camera_id", cameraId);
    yamlWriteString(out, "model", model);
    yamlWriteInt(out, "pattern_cols", pattern.width);
    yamlWriteInt(out, "pattern_rows", pattern.height);
    yamlWriteDouble(out, "square_mm", squareMm);
    yamlWriteInt(out, "image_width", imageSize.width);
    yamlWriteInt(out, "image_height", imageSize.height);
    yamlWriteInt(out, "used_images", (int)kept.size());
    yamlWriteInt(out, "rejected_images", (int)rejected.size());
    yamlWriteDouble(out, "rms_solver_px", rms);
    yamlWriteDouble(out, "mean_reproj_px", mean(perErr));
    yamlWriteDouble(out, "max_reproj_px", maxErr);

    // Keys expected by extrinsic/main.cpp
    yamlWriteOpenCvMatrix(out, "camera_matrix", K);
    yamlWriteOpenCvMatrix(out, "distortion_coefficients", D);

    out << "per_image_reproj_error_px:\n";
    for (size_t j = 0; j < kept.size(); ++j) {
        const int idx = kept[j];
        const std::string name = imagePaths[(size_t)idx].filename().string();
        out.setf(std::ios::fixed);
        out.precision(6);
        out << "  " << name << ": " << perErr[j] << "\n";
    }

    if (!rejected.empty()) {
        out << "rejected_images_detail:\n";
        for (const auto& r : rejected) {
            out.setf(std::ios::fixed);
            out.precision(6);
            out << "  - { file: \"" << r.path.filename().string() << "\", error_px: " << r.errorPx << " }\n";
        }
    }

    out.close();
    std::cout << "Saved: " << fs::absolute(output).string() << "\n";
    return 0;
}
