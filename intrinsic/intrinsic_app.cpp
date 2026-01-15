#include "intrinsic_app.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace fs = std::filesystem;

namespace intrin {

using std::cerr;
using std::cout;

static std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    return s;
}

static bool parsePattern(const std::string& s, cv::Size& pattern) {
    const auto x = s.find('x');
    if (x == std::string::npos) return false;
    try {
        const int w = std::stoi(s.substr(0, x));
        const int h = std::stoi(s.substr(x + 1));
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
    const fs::path cwd = fs::current_path();
    if (fs::exists(cwd / "data") && fs::is_directory(cwd / "data")) return cwd;

    fs::path exe = executablePath();
    if (!exe.empty()) {
        fs::path dir = exe.parent_path();
        for (int i = 0; i < 6 && !dir.empty(); ++i) {
            if (fs::exists(dir / "data") && fs::is_directory(dir / "data")) return dir;
            dir = dir.parent_path();
        }
    }

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

static double blurScoreVarianceOfLaplacian(const cv::Mat& gray) {
    cv::Mat lap;
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev[0] * stddev[0];
}

static double boardAreaRatioBbox(const std::vector<cv::Point2f>& corners, int w, int h) {
    if (corners.empty() || w <= 0 || h <= 0) return 0.0;
    const cv::Rect bbox = cv::boundingRect(corners);
    const double imgArea = (double)w * (double)h;
    if (imgArea <= 0.0) return 0.0;
    return (double)bbox.area() / imgArea;
}

static bool detectChessboardCornersRobust(const cv::Mat& gray,
                                         const cv::Size& pattern,
                                         std::vector<cv::Point2f>& corners,
                                         std::string& detector) {
    corners.clear();
    detector.clear();

#if (CV_VERSION_MAJOR >= 4)
    {
        const int flagsSb = cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY;
        if (cv::findChessboardCornersSB(gray, pattern, corners, flagsSb)) {
            detector = "sb";
            return true;
        }
    }
#endif

    {
        const int flagsClassic = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
        if (cv::findChessboardCorners(gray, pattern, corners, flagsClassic)) {
            detector = "classic";
            return true;
        }

        cv::Mat eq;
        cv::equalizeHist(gray, eq);
        if (cv::findChessboardCorners(eq, pattern, corners, flagsClassic)) {
            detector = "classic_eq";
            return true;
        }
    }

    return false;
}

static double mean(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

static double perImageReprojError(const std::vector<cv::Point3f>& obj,
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

static void yamlWriteDouble(std::ostream& os, const std::string& key, double value, int precision = 10) {
    os.setf(std::ios::fixed);
    os.precision(precision);
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

static void validateAndWarn(const cv::Mat& K, const cv::Mat& D, const cv::Size& imageSize, bool fisheye) {
    if (K.empty() || D.empty() || imageSize.empty()) return;

    const double fx = K.at<double>(0, 0);
    const double fy = K.at<double>(1, 1);

    const double w = static_cast<double>(imageSize.width);
    const double h = static_cast<double>(imageSize.height);

    if (fx < 0.3 * w || fx > 3.0 * w) {
        cerr << "WARNING: fx out of plausible range: " << fx << " (w=" << w << ")\n";
    }
    if (fy < 0.3 * h || fy > 3.0 * h) {
        cerr << "WARNING: fy out of plausible range: " << fy << " (h=" << h << ")\n";
    }

    if (!fisheye) {
        if (D.total() >= 1) {
            const double k1 = D.at<double>(0);
            if (std::abs(k1) > 2.0) cerr << "WARNING: |k1| looks large: " << k1 << "\n";
        }
        if (D.total() >= 3) {
            const double p1 = D.at<double>(2);
            if (std::abs(p1) > 0.5) cerr << "WARNING: |p1| looks large: " << p1 << "\n";
        }
    }
}

void printUsage() {
    cout
        << "intrinsic (offline)\n"
        << "Usage:\n"
        << "  intrinsic.exe --camera-id calib_cam1 [--pattern 8x5] [--square-mm 65] [--model standard]\n"
        << "  intrinsic.exe --images-dir data/calib_cam1 [--pattern 8x5] [--square-mm 65] [--model fisheye]\n"
        << "\n"
        << "Options:\n"
        << "  --camera-id <name>               Uses images in data/<name>/\n"
        << "  --images-dir <dir>               Directory with calibration images (absolute or relative)\n"
        << "  --ext <png|jpg>                  Extension filter (default png)\n"
        << "  --pattern <WxH>                  Inner corners, e.g. 8x5 (default 8x5)\n"
        << "  --square-mm <float>              Square size in mm (default 65)\n"
        << "  --model <standard|fisheye>       Lens model (default standard)\n"
        << "  --min-detections <int>           Minimum valid detections required (default 10)\n"
        << "  --max-per-image-error <float>    Reject images above this reproj error (px) (default 1.2)\n"
        << "  --max-remove <int>               Max outliers to remove (default 15; clamped internally)\n"
        << "  --min-board-area-ratio <float>   Reject boards that are too small (default 0.10)\n"
        << "  --output <file.yml>              Output YAML file (default: <images-dir>/intrinsic.yml)\n"
        << "  --report <file.csv>              Write detection report CSV (found/blur/area, etc)\n"
        << "  --quiet                          Less console output\n";
}

bool parseArgs(int argc, char** argv, AppConfig& cfg) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + name);
            return argv[++i];
        };

        if (a == "--help" || a == "-h") {
            printUsage();
            return false;
        } else if (a == "--camera-id") {
            cfg.cameraId = need("--camera-id");
        } else if (a == "--images-dir") {
            cfg.imagesDir = need("--images-dir");
        } else if (a == "--ext") {
            cfg.extLower = toLower(need("--ext"));
        } else if (a == "--pattern") {
            cv::Size p;
            if (!parsePattern(need("--pattern"), p)) throw std::runtime_error("Bad --pattern; expected WxH like 8x5");
            cfg.pattern = p;
        } else if (a == "--square-mm") {
            cfg.squareMm = std::stod(need("--square-mm"));
        } else if (a == "--model") {
            cfg.model = toLower(need("--model"));
            if (cfg.model != "standard" && cfg.model != "fisheye") {
                throw std::runtime_error("--model must be 'standard' or 'fisheye'");
            }
        } else if (a == "--min-detections") {
            cfg.minDetections = std::stoi(need("--min-detections"));
        } else if (a == "--max-per-image-error") {
            cfg.maxPerImageErrorPx = std::stod(need("--max-per-image-error"));
        } else if (a == "--max-remove") {
            cfg.maxRemove = std::stoi(need("--max-remove"));
        } else if (a == "--min-board-area-ratio") {
            cfg.minBoardAreaRatio = std::stod(need("--min-board-area-ratio"));
        } else if (a == "--output") {
            cfg.output = need("--output");
        } else if (a == "--report") {
            cfg.reportCsv = need("--report");
        } else if (a == "--quiet") {
            cfg.verbose = false;
        } else {
            throw std::runtime_error(std::string("Unknown arg: ") + a);
        }
    }

    if (cfg.cameraId.empty() && cfg.imagesDir.empty()) {
        throw std::runtime_error("Provide --camera-id or --images-dir");
    }

    return true;
}

int run(const AppConfig& cfgIn) {
    AppConfig cfg = cfgIn;

    const fs::path dataRoot = findDataRoot();

    if (cfg.imagesDir.empty()) {
        cfg.imagesDir = dataRoot / "data" / cfg.cameraId;
    } else if (cfg.imagesDir.is_relative()) {
        if (fs::exists(dataRoot / cfg.imagesDir)) {
            cfg.imagesDir = dataRoot / cfg.imagesDir;
        } else {
            cfg.imagesDir = dataRoot / "data" / cfg.imagesDir;
        }
    }

    if (cfg.output.empty()) {
        cfg.output = cfg.imagesDir / "intrinsic.yml";
    } else if (cfg.output.is_relative()) {
        cfg.output = dataRoot / cfg.output;
    }

    if (!fs::exists(cfg.imagesDir) || !fs::is_directory(cfg.imagesDir)) {
        cerr << "ERROR: Missing images directory: " << cfg.imagesDir.string() << "\n";
        return 1;
    }

    const bool useFisheye = (cfg.model == "fisheye");
    const auto files = listImages(cfg.imagesDir, cfg.extLower);
    if (files.empty()) {
        cerr << "ERROR: No images found in " << cfg.imagesDir.string() << " with extension " << cfg.extLower << "\n";
        return 1;
    }

    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<fs::path> imagePaths;
    cv::Size imageSize;

    struct DetRow {
        std::string file;
        bool loaded = false;
        bool found = false;
        bool accepted = false;
        std::string detector;
        double blur = 0.0;
        double areaRatio = 0.0;
        int w = 0;
        int h = 0;
    };
    std::vector<DetRow> report;
    report.reserve(files.size());

    if (cfg.verbose) {
        cout << "Build: " << __DATE__ << " " << __TIME__ << "\n";
        cout << "Images dir: " << cfg.imagesDir.string() << "\n";
        cout << "Found " << files.size() << " images (" << cfg.extLower << ")\n";
        cout << "Detecting chessboards...\n";
    }

    for (const auto& p : files) {
        DetRow row;
        row.file = p.filename().string();

        cv::Mat img = cv::imread(p.string(), cv::IMREAD_COLOR);
        row.loaded = !img.empty();
        if (!row.loaded) {
            report.push_back(std::move(row));
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        row.w = gray.cols;
        row.h = gray.rows;
        row.blur = blurScoreVarianceOfLaplacian(gray);

        if (imageSize.empty()) {
            imageSize = gray.size();
        } else if (gray.size() != imageSize) {
            cerr << "ERROR: Mixed image sizes. Found " << gray.cols << "x" << gray.rows
                 << " but expected " << imageSize.width << "x" << imageSize.height << "\n";
            return 1;
        }

        std::vector<cv::Point2f> corners;
        std::string detector;
        const bool found = detectChessboardCornersRobust(gray, cfg.pattern, corners, detector);
        row.found = found;
        row.detector = detector;
        if (!found) {
            report.push_back(std::move(row));
            continue;
        }

        cv::cornerSubPix(
            gray,
            corners,
            cv::Size(15, 15),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 80, 1e-12));

        row.areaRatio = boardAreaRatioBbox(corners, gray.cols, gray.rows);
        if (cfg.minBoardAreaRatio > 0.0 && row.areaRatio < cfg.minBoardAreaRatio) {
            report.push_back(std::move(row));
            continue;
        }

        row.accepted = true;

        imagePoints.push_back(std::move(corners));
        imagePaths.push_back(p);
        report.push_back(std::move(row));
    }

    if (!cfg.reportCsv.empty()) {
        fs::path rp = cfg.reportCsv;
        if (rp.is_relative()) rp = dataRoot / rp;
        if (!rp.parent_path().empty()) fs::create_directories(rp.parent_path());
        std::ofstream csv(rp);
        if (csv.is_open()) {
            csv << "file,loaded,found,accepted,detector,blur,area_ratio,width,height\n";
            csv.setf(std::ios::fixed);
            csv << std::setprecision(6);
            for (const auto& r : report) {
                csv << '"' << r.file << '"' << ','
                    << (r.loaded ? 1 : 0) << ','
                    << (r.found ? 1 : 0) << ','
                    << (r.accepted ? 1 : 0) << ','
                    << '"' << r.detector << '"' << ','
                    << r.blur << ','
                    << r.areaRatio << ','
                    << r.w << ','
                    << r.h << "\n";
            }
            csv.close();
            if (cfg.verbose) cout << "Wrote report: " << fs::absolute(rp).string() << "\n";
        } else {
            cerr << "WARNING: Could not write report CSV: " << fs::absolute(rp).string() << "\n";
        }
    }

    if (cfg.verbose) {
        int loadedN = 0, foundN = 0, acceptedN = 0;
        for (const auto& r : report) {
            loadedN += r.loaded ? 1 : 0;
            foundN += r.found ? 1 : 0;
            acceptedN += r.accepted ? 1 : 0;
        }
        cout << "Detection summary: loaded=" << loadedN << "/" << report.size()
             << " found=" << foundN << "/" << report.size()
             << " accepted=" << acceptedN << "/" << report.size()
             << " (min_board_area_ratio=" << cfg.minBoardAreaRatio << ")\n";
    }

    if (static_cast<int>(imagePoints.size()) < cfg.minDetections) {
        cerr << "ERROR: Not enough valid detections: " << imagePoints.size() << " (need >= " << cfg.minDetections << ")\n";
        return 1;
    }

    // Object points template
    std::vector<cv::Point3f> obj;
    obj.reserve(static_cast<size_t>(cfg.pattern.width) * static_cast<size_t>(cfg.pattern.height));
    for (int y = 0; y < cfg.pattern.height; ++y) {
        for (int x = 0; x < cfg.pattern.width; ++x) {
            obj.emplace_back(static_cast<float>(x * cfg.squareMm), static_cast<float>(y * cfg.squareMm), 0.0f);
        }
    }

    std::vector<int> kept(static_cast<int>(imagePoints.size()));
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
            imgPts.push_back(imagePoints[static_cast<size_t>(i)]);
        }

        rvecs.clear();
        tvecs.clear();

        const cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 500, 1e-12);

        double rms = 0.0;
        if (useFisheye) {
            K = cv::Mat::zeros(3, 3, CV_64F);
            D = cv::Mat::zeros(4, 1, CV_64F);
            // NOTE: CALIB_CHECK_COND can throw for small/degenerate datasets; keep defaults more permissive.
            const int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
                              cv::fisheye::CALIB_FIX_SKEW;
            rms = cv::fisheye::calibrate(objectPoints, imgPts, imageSize, K, D, rvecs, tvecs, flags, criteria);
        } else {
            // Brown–Conrady 5 params: k1 k2 p1 p2 k3 (avoid rational / overfitting)
            D = cv::Mat::zeros(5, 1, CV_64F);
            const int flags = cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6;
            rms = cv::calibrateCamera(objectPoints, imgPts, imageSize, K, D, rvecs, tvecs, flags, criteria);
        }

        perErr.clear();
        perErr.reserve(idxs.size());
        for (size_t j = 0; j < idxs.size(); ++j) {
            const int idx = idxs[j];
            perErr.push_back(perImageReprojError(obj, imagePoints[static_cast<size_t>(idx)], rvecs[j], tvecs[j], K, D, useFisheye));
        }

        return rms;
    };

    cv::Mat K, D;
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> perErr;

    const int maxRemoveClamped = std::min(cfg.maxRemove, 5);
    double lastRms = std::numeric_limits<double>::infinity();
    double rms = 0.0;

    for (int iter = 0; iter <= maxRemoveClamped; ++iter) {
        try {
            rms = runCalib(kept, K, D, rvecs, tvecs, perErr);
        } catch (const cv::Exception& e) {
            cerr << "ERROR: OpenCV calibration failed: " << e.what() << "\n";
            if (useFisheye) {
                cerr << "Hint: fisheye calibration needs diverse views (tilt/roll, board near corners, varying distance).\n";
                cerr << "      Current detected images: " << kept.size() << " (try capturing more usable frames).\n";
            }
            return 1;
        }

        const auto itWorst = std::max_element(perErr.begin(), perErr.end());
        const double worst = (itWorst != perErr.end()) ? *itWorst : 0.0;
        const int worstPos = (itWorst != perErr.end()) ? static_cast<int>(std::distance(perErr.begin(), itWorst)) : -1;

        if (cfg.verbose) {
            cout << "Iter " << iter << ": kept=" << kept.size() << " rms=" << rms
                 << " mean=" << mean(perErr) << " max=" << worst << "\n";
        }

        // Stop if RMS is no longer improving meaningfully.
        if (iter > 0 && (lastRms - rms) < cfg.minRmsImprovement) {
            break;
        }
        lastRms = rms;

        // Remove worst if it exceeds threshold, but keep dataset reasonably sized.
        if (worstPos >= 0 && worst > cfg.maxPerImageErrorPx &&
            static_cast<int>(kept.size()) > std::max(cfg.minDetections, cfg.minDatasetForPrune)) {
            const int worstIdx = kept[static_cast<size_t>(worstPos)];
            rejected.push_back({imagePaths[static_cast<size_t>(worstIdx)], worst});
            kept.erase(kept.begin() + worstPos);
            continue;
        }

        break;
    }

    const double maxErr = perErr.empty() ? 0.0 : *std::max_element(perErr.begin(), perErr.end());

    cout << "\n=== FINAL INTRINSICS ===\n";
    cout << "Model:        " << cfg.model << "\n";
    cout << "Used images:  " << kept.size() << " (rejected " << rejected.size() << ")\n";
    cout << "RMS (solver): " << rms << " px\n";
    cout << "Mean reproj:  " << mean(perErr) << " px\n";
    cout << "Max reproj:   " << maxErr << " px\n";
    cout << "Image size:   " << imageSize.width << "x" << imageSize.height << "\n";

    validateAndWarn(K, D, imageSize, useFisheye);

    fs::create_directories(cfg.output.parent_path());
    std::ofstream out(cfg.output);
    if (!out.is_open()) {
        cerr << "ERROR: Cannot write " << fs::absolute(cfg.output).string() << "\n";
        return 1;
    }

    out << "%YAML:1.0\n---\n";
    yamlWriteString(out, "build", std::string(__DATE__) + " " + __TIME__);
    if (!cfg.cameraId.empty()) yamlWriteString(out, "camera_id", cfg.cameraId);
    yamlWriteString(out, "model", cfg.model);
    yamlWriteInt(out, "pattern_cols", cfg.pattern.width);
    yamlWriteInt(out, "pattern_rows", cfg.pattern.height);
    yamlWriteDouble(out, "square_mm", cfg.squareMm);
    yamlWriteInt(out, "image_width", imageSize.width);
    yamlWriteInt(out, "image_height", imageSize.height);
    yamlWriteInt(out, "used_images", static_cast<int>(kept.size()));
    yamlWriteInt(out, "rejected_images", static_cast<int>(rejected.size()));
    yamlWriteDouble(out, "rms_solver_px", rms);
    yamlWriteDouble(out, "mean_reproj_px", mean(perErr));
    yamlWriteDouble(out, "max_reproj_px", maxErr);

    // Keys expected by extrinsic tool
    yamlWriteOpenCvMatrix(out, "camera_matrix", K);
    yamlWriteOpenCvMatrix(out, "distortion_coefficients", D);

    out << "per_image_reproj_error_px:\n";
    for (size_t j = 0; j < kept.size(); ++j) {
        const int idx = kept[j];
        const std::string name = imagePaths[static_cast<size_t>(idx)].filename().string();
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
    cout << "Saved: " << fs::absolute(cfg.output).string() << "\n";
    return 0;
}

} // namespace intrin
