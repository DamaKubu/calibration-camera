#include "live_motion_app.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "../capture/mf_camera.h"

namespace fs = std::filesystem;

namespace livemotion {

using std::cerr;
using std::cout;

static fs::path defaultProjectRoot() {
    fs::path start = fs::current_path();
    fs::path cmake = mfcam::findUpwardsForFile(start, "CMakeLists.txt");
    if (!cmake.empty()) return cmake.parent_path();
    return start;
}

static fs::path resolvePathSmart(const fs::path& root, const fs::path& p) {
    if (p.empty()) return {};
    if (p.is_absolute()) return p;

    // 1) relative to current working directory
    if (fs::exists(p)) return fs::absolute(p);

    // 2) relative to repo root
    if (fs::exists(root / p)) return fs::absolute(root / p);

    // 3) common default folder for stereo extrinsics
    if (p.has_filename() && fs::exists(root / "data" / "pairs_cam1_cam2" / p.filename())) {
        return fs::absolute(root / "data" / "pairs_cam1_cam2" / p.filename());
    }

    return fs::absolute(root / p);
}

static std::string nowIso8601() {
    using clock = std::chrono::system_clock;
    const auto now = clock::now();
    const std::time_t t = clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    tm = *std::localtime(&t);
#endif
    std::ostringstream ss;
    ss << std::setw(4) << std::setfill('0') << (tm.tm_year + 1900) << "-"
       << std::setw(2) << std::setfill('0') << (tm.tm_mon + 1) << "-"
       << std::setw(2) << std::setfill('0') << tm.tm_mday << "T"
       << std::setw(2) << std::setfill('0') << tm.tm_hour << ":"
       << std::setw(2) << std::setfill('0') << tm.tm_min << ":"
       << std::setw(2) << std::setfill('0') << tm.tm_sec;
    return ss.str();
}

static bool loadIntrinsics(const fs::path& path, cv::Mat& K, cv::Mat& D, cv::Size& imageSize, std::string& model) {
    cv::FileStorage fs(path.string(), cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;

    int w = 0, h = 0;
    if (!fs["image_width"].empty()) fs["image_width"] >> w;
    if (!fs["image_height"].empty()) fs["image_height"] >> h;
    imageSize = (w > 0 && h > 0) ? cv::Size(w, h) : cv::Size();

    if (!fs["model"].empty()) fs["model"] >> model;

    return !K.empty() && !D.empty();
}

static bool loadStereoExtrinsics(const fs::path& path, cv::Mat& R, cv::Mat& T, bool& fisheye) {
    cv::FileStorage fs(path.string(), cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    fs["R"] >> R;
    fs["T"] >> T;

    int fe = 0;
    if (!fs["fisheye"].empty()) fs["fisheye"] >> fe;
    fisheye = (fe != 0);

    return !R.empty() && !T.empty();
}

static int flagsForDistortion(const cv::Mat& D) {
    const int n = (int)D.total();
    int flags = 0;
    if (n >= 8) flags |= cv::CALIB_RATIONAL_MODEL;
    if (n >= 12) flags |= cv::CALIB_THIN_PRISM_MODEL;
    if (n >= 14) flags |= cv::CALIB_TILTED_MODEL;
    return flags;
}

struct BoardDet {
    bool found = false;
    std::vector<cv::Point2f> corners;
    double meanMotionPx = 1e9;
    double areaRatio = 0.0;
};

static double boardAreaRatioBbox(const std::vector<cv::Point2f>& corners, int w, int h) {
    if (corners.empty() || w <= 0 || h <= 0) return 0.0;
    const cv::Rect bbox = cv::boundingRect(corners);
    const double imgArea = (double)w * (double)h;
    if (imgArea <= 0.0) return 0.0;
    return (double)bbox.area() / imgArea;
}

static bool detectChessboardCornersRobust(const cv::Mat& gray,
                                         const cv::Size& pattern,
                                         std::vector<cv::Point2f>& corners) {
    corners.clear();

#if (CV_VERSION_MAJOR >= 4)
    {
        const int flagsSb = cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY;
        if (cv::findChessboardCornersSB(gray, pattern, corners, flagsSb)) return true;
    }
#endif

    {
        const int flagsClassic = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
        if (cv::findChessboardCorners(gray, pattern, corners, flagsClassic)) return true;

        cv::Mat eq;
        cv::equalizeHist(gray, eq);
        if (cv::findChessboardCorners(eq, pattern, corners, flagsClassic)) return true;
    }

    return false;
}

static BoardDet detectBoard(const cv::Mat& bgr, const cv::Size& pattern, const std::vector<cv::Point2f>* prevCorners) {
    BoardDet out;
    if (bgr.empty()) return out;

    cv::Mat gray;
    if (bgr.channels() == 3) cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    else if (bgr.channels() == 4) cv::cvtColor(bgr, gray, cv::COLOR_BGRA2GRAY);
    else gray = bgr;

    std::vector<cv::Point2f> corners;
    if (!detectChessboardCornersRobust(gray, pattern, corners)) return out;

    cv::cornerSubPix(gray, corners, cv::Size(7, 7), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 60, 1e-3));

    out.found = true;
    out.corners = corners;
    out.areaRatio = boardAreaRatioBbox(corners, gray.cols, gray.rows);

    if (prevCorners && prevCorners->size() == corners.size()) {
        double sum = 0.0;
        for (size_t i = 0; i < corners.size(); ++i) sum += cv::norm(corners[i] - (*prevCorners)[i]);
        out.meanMotionPx = corners.empty() ? 1e9 : (sum / (double)corners.size());
    } else {
        out.meanMotionPx = 0.0;
    }

    return out;
}

static std::vector<cv::Point3f> makeObjectPoints(const cv::Size& pattern, double squareMm) {
    std::vector<cv::Point3f> obj;
    obj.reserve((size_t)pattern.area());
    for (int y = 0; y < pattern.height; ++y) {
        for (int x = 0; x < pattern.width; ++x) {
            obj.emplace_back((float)(x * squareMm), (float)(y * squareMm), 0.0f);
        }
    }
    return obj;
}

static void saveStereoExtrinsicsYml(const fs::path& outPath,
                                   const fs::path& pairsDir,
                                   const fs::path& intr1,
                                   const fs::path& intr2,
                                   const cv::Size& pattern,
                                   double squareMm,
                                   int usedPairs,
                                   bool fisheye,
                                   double rms,
                                   const cv::Mat& R,
                                   const cv::Mat& T,
                                   const cv::Mat& E,
                                   const cv::Mat& F) {
    if (outPath.empty()) return;
    if (!outPath.parent_path().empty()) fs::create_directories(outPath.parent_path());

    cv::FileStorage fsOut(outPath.string(), cv::FileStorage::WRITE);
    fsOut << "build" << (std::string(__DATE__) + " " + __TIME__);
    fsOut << "pairs_dir" << pairsDir.string();
    fsOut << "intr1" << intr1.string();
    fsOut << "intr2" << intr2.string();
    fsOut << "pattern_cols" << pattern.width;
    fsOut << "pattern_rows" << pattern.height;
    fsOut << "square_mm" << squareMm;
    fsOut << "used_pairs" << usedPairs;
    fsOut << "fisheye" << (fisheye ? 1 : 0);
    fsOut << "rms" << rms;
    fsOut << "R" << R;
    fsOut << "T" << T;
    if (!E.empty()) fsOut << "E" << E;
    if (!F.empty()) fsOut << "F" << F;
    fsOut.release();
}

struct BallObs {
    bool found = false;
    cv::Point2f centerPx{};
    double areaPx = 0.0;
};

static void undistortPointNormalized(const cv::Point2f& p,
                                    const cv::Mat& K,
                                    const cv::Mat& D,
                                    bool fisheye,
                                    cv::Point2f& outNorm) {
    std::vector<cv::Point2f> src{p};
    std::vector<cv::Point2f> dst;
    if (fisheye) {
        cv::fisheye::undistortPoints(src, dst, K, D);
    } else {
        cv::undistortPoints(src, dst, K, D);
    }
    outNorm = dst.empty() ? cv::Point2f() : dst[0];
}

static bool triangulateOne(const cv::Point2f& p1Norm,
                          const cv::Point2f& p2Norm,
                          const cv::Mat& R,
                          const cv::Mat& T,
                          cv::Point3d& outX) {
    cv::Mat P1 = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat P2 = cv::Mat::zeros(3, 4, CV_64F);

    // P1 = [I|0]
    P1.at<double>(0, 0) = 1.0;
    P1.at<double>(1, 1) = 1.0;
    P1.at<double>(2, 2) = 1.0;

    // P2 = [R|T]
    cv::Mat R64, T64;
    R.convertTo(R64, CV_64F);
    T.convertTo(T64, CV_64F);
    R64.copyTo(P2(cv::Rect(0, 0, 3, 3)));
    T64.copyTo(P2(cv::Rect(3, 0, 1, 3)));

    cv::Mat pts1(2, 1, CV_64F);
    cv::Mat pts2(2, 1, CV_64F);
    pts1.at<double>(0, 0) = p1Norm.x;
    pts1.at<double>(1, 0) = p1Norm.y;
    pts2.at<double>(0, 0) = p2Norm.x;
    pts2.at<double>(1, 0) = p2Norm.y;

    cv::Mat X;
    cv::triangulatePoints(P1, P2, pts1, pts2, X);
    if (X.empty() || X.rows != 4) return false;

    const double w = X.at<double>(3, 0);
    if (std::abs(w) < 1e-12) return false;

    outX.x = X.at<double>(0, 0) / w;
    outX.y = X.at<double>(1, 0) / w;
    outX.z = X.at<double>(2, 0) / w;
    return true;
}

void printUsage(std::ostream& os) {
    os << "live_motion (stereo chessboard tracking)\n\n"
       << "Uses cam1+cam2 to track a chessboard center and triangulate 3D position.\n\n"
       << "Usage:\n"
       << "  live_motion.exe --cam1 cam1 --cam2 cam2 --extrinsics data/pairs_cam1_cam2/stereo_extrinsic.yml\n"
       << "\nOptions:\n"
       << "  --cameras-yml <path>          cameras.yml path (default cameras.yml)\n"
       << "  --cam1 <key>                  Camera key (default cam1)\n"
       << "  --cam2 <key>                  Camera key (default cam2)\n"
       << "  --intrinsics-root <dir>       Root folder for calib_*/intrinsic.yml (default data)\n"
       << "  --intrinsics-prefix <prefix>  Intrinsics folder prefix (default calib_)\n"
    << "  --extrinsics <file.yml>       Stereo extrinsics file (default data/pairs_cam1_cam2/stereo_extrinsic.yml)\n"
    << "  --quick-calib                 Quick stereo calibration first (then track)\n"
    << "  --calib-frames <n>            Number of good chessboard pairs to collect (default 25)\n"
    << "  --pattern <WxH>               Chessboard inner corners (default 9x6)\n"
    << "  --square-mm <float>           Chessboard square size in mm (default 23.6)\n"
     << "  --save-extrinsics <file.yml>  Save computed stereo extrinsics\n"
     << "  --baseline-mm <float>         Constrain baseline length (scale T)\n"
     << "  --force-stereo                Force R=I and T=[baseline,0,0]\n"
     << "  --board                       Track chessboard (default)\n"
     << "  --board-skip <n>              Run board detection every N frames (default 0)\n"
     << "  --det-scale <f>               Downscale factor for detection (e.g. 0.5)\n"
       << "  --csv <file.csv>              Write CSV: time,x,y,z,u1,v1,u2,v2\n"
       << "  --no-show                     Disable preview windows\n"
       << "  --max-fps <n>                 Limit processing rate\n"
       << "  --req-width <n>               Request capture width\n"
       << "  --req-height <n>              Request capture height\n"
       << "  --req-fps <n>                 Request capture fps\n";
}

bool parseArgs(int argc, char** argv, AppConfig& cfg) {
    auto need = [&](int& i, const char* name) -> std::string {
        if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + name);
        return argv[++i];
    };

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            printUsage(std::cout);
            return false;
        } else if (a == "--cameras-yml") {
            cfg.camerasYml = need(i, "--cameras-yml");
        } else if (a == "--cam1") {
            cfg.cam1Key = need(i, "--cam1");
        } else if (a == "--cam2") {
            cfg.cam2Key = need(i, "--cam2");
        } else if (a == "--intrinsics-root") {
            cfg.intrinsicsRoot = need(i, "--intrinsics-root");
        } else if (a == "--intrinsics-prefix") {
            cfg.intrinsicsPrefix = need(i, "--intrinsics-prefix");
        } else if (a == "--extrinsics") {
            cfg.stereoExtrinsics = need(i, "--extrinsics");
        } else if (a == "--quick-calib") {
            cfg.quickCalibrateStereo = true;
        } else if (a == "--calib-frames") {
            cfg.calibFrames = std::stoi(need(i, "--calib-frames"));
        } else if (a == "--pattern") {
            const std::string s = need(i, "--pattern");
            const auto x = s.find('x');
            if (x == std::string::npos) throw std::runtime_error("Bad --pattern; expected WxH like 8x5");
            cfg.patternCols = std::stoi(s.substr(0, x));
            cfg.patternRows = std::stoi(s.substr(x + 1));
        } else if (a == "--square-mm") {
            cfg.squareMm = std::stod(need(i, "--square-mm"));
        } else if (a == "--save-extrinsics") {
            cfg.saveStereoExtrinsics = need(i, "--save-extrinsics");
        } else if (a == "--baseline-mm") {
            cfg.baselineMm = std::stod(need(i, "--baseline-mm"));
        } else if (a == "--force-stereo") {
            cfg.forceStereo = true;
        } else if (a == "--board" || a == "--board-only") {
            cfg.trackBoard = true;
        } else if (a == "--board-skip") {
            cfg.boardSkip = std::max(0, std::stoi(need(i, "--board-skip")));
        } else if (a == "--det-scale") {
            cfg.detScale = std::stod(need(i, "--det-scale"));
            if (cfg.detScale <= 0.0 || cfg.detScale > 1.0) throw std::runtime_error("--det-scale must be in (0,1]");
        } else if (a == "--csv") {
            cfg.outputCsv = need(i, "--csv");
        } else if (a == "--no-show") {
            cfg.show = false;
        } else if (a == "--max-fps") {
            cfg.maxFps = std::stoi(need(i, "--max-fps"));
        } else if (a == "--req-width") {
            cfg.reqWidth = (unsigned)std::stoul(need(i, "--req-width"));
        } else if (a == "--req-height") {
            cfg.reqHeight = (unsigned)std::stoul(need(i, "--req-height"));
        } else if (a == "--req-fps") {
            cfg.reqFps = (unsigned)std::stoul(need(i, "--req-fps"));
        } else {
            throw std::runtime_error(std::string("Unknown arg: ") + a);
        }
    }

    return true;
}

int run(const AppConfig& cfg) {
    const fs::path root = defaultProjectRoot();

    fs::path camerasYml = cfg.camerasYml;
    if (camerasYml.is_relative()) camerasYml = root / camerasYml;

    fs::path intr1 = root / cfg.intrinsicsRoot / (cfg.intrinsicsPrefix + cfg.cam1Key) / "intrinsic.yml";
    fs::path intr2 = root / cfg.intrinsicsRoot / (cfg.intrinsicsPrefix + cfg.cam2Key) / "intrinsic.yml";

    fs::path ext = resolvePathSmart(root, cfg.stereoExtrinsics);

    cout << "Project root: " << root.string() << "\n";
    cout << "Intr1: " << intr1.string() << "\n";
    cout << "Intr2: " << intr2.string() << "\n";
    cout << "Extrinsics: " << ext.string() << "\n";

    cv::Mat K1, D1, K2, D2;
    cv::Size size1, size2;
    std::string model1, model2;

    if (!loadIntrinsics(intr1, K1, D1, size1, model1)) {
        throw std::runtime_error("Failed to load intrinsics cam1: " + intr1.string());
    }
    if (!loadIntrinsics(intr2, K2, D2, size2, model2)) {
        throw std::runtime_error("Failed to load intrinsics cam2: " + intr2.string());
    }

    cv::Mat R, T;
    bool fisheyeStereo = false;
    bool haveStereo = false;
    if (!cfg.quickCalibrateStereo) {
        if (cfg.forceStereo) {
            if (cfg.baselineMm <= 0.0) {
                throw std::runtime_error("--force-stereo requires --baseline-mm > 0");
            }
            R = cv::Mat::eye(3, 3, CV_64F);
            T = (cv::Mat_<double>(3, 1) << cfg.baselineMm, 0.0, 0.0);
            fisheyeStereo = false;
            haveStereo = true;
            cout << "Using forced stereo: R=I, T=[" << cfg.baselineMm << ",0,0] mm\n";
        } else {
            if (!loadStereoExtrinsics(ext, R, T, fisheyeStereo)) {
                throw std::runtime_error("Failed to load stereo extrinsics: " + ext.string());
            }
            if (cfg.baselineMm > 0.0) {
                const double tnorm = cv::norm(T);
                if (tnorm > 1e-9) {
                    const double scale = cfg.baselineMm / tnorm;
                    T *= scale;
                    cout << "Applied baseline constraint to loaded extrinsics: " << cfg.baselineMm << " mm (scale=" << scale << ")\n";
                } else {
                    cout << "Baseline constraint requested, but |T| ~ 0; skipped.\n";
                }
            }
            haveStereo = true;
        }
    }

    const bool fisheye = fisheyeStereo || (model1 == "fisheye") || (model2 == "fisheye");
    if (fisheye) {
        cout << "Using fisheye undistortion\n";
    } else {
        cout << "Using standard undistortion\n";
    }

    std::string link1, link2;
    if (!mfcam::loadSymbolicLinkFromCamerasYml(camerasYml, cfg.cam1Key, link1)) {
        throw std::runtime_error("Failed to find cam1 key in cameras.yml: " + cfg.cam1Key);
    }
    if (!mfcam::loadSymbolicLinkFromCamerasYml(camerasYml, cfg.cam2Key, link2)) {
        throw std::runtime_error("Failed to find cam2 key in cameras.yml: " + cfg.cam2Key);
    }

    mfcam::OpenOptions opts;
    opts.width = cfg.reqWidth;
    opts.height = cfg.reqHeight;
    opts.fps = cfg.reqFps;

    mfcam::Camera cam1;
    mfcam::Camera cam2;
    if (!cam1.openSymbolicLink(link1, opts)) throw std::runtime_error("Failed to open cam1");
    if (!cam2.openSymbolicLink(link2, opts)) throw std::runtime_error("Failed to open cam2");

    if (cfg.quickCalibrateStereo) {
        const cv::Size pattern(cfg.patternCols, cfg.patternRows);
        if (pattern.width <= 0 || pattern.height <= 0) throw std::runtime_error("Bad pattern size");
        if (cfg.calibFrames < 5) throw std::runtime_error("--calib-frames too small (need >= 5)");

        const std::vector<cv::Point3f> obj = makeObjectPoints(pattern, cfg.squareMm);
        std::vector<std::vector<cv::Point3f>> objectPoints;
        std::vector<std::vector<cv::Point2f>> img1;
        std::vector<std::vector<cv::Point2f>> img2;
        objectPoints.reserve((size_t)cfg.calibFrames);
        img1.reserve((size_t)cfg.calibFrames);
        img2.reserve((size_t)cfg.calibFrames);

        std::vector<cv::Point2f> prev1, prev2;
        int stableCount = 0;
        // Loosen gating criteria
        const double maxMotionPx = 1.5;      // allow slightly more motion
        const double minAreaRatio = 0.02;    // allow very small boards in frame
        const int requiredStable = 2;        // fewer frames needed for capture

        // Frame rate limiter setup
        auto lastFrame = std::chrono::high_resolution_clock::now();
        const double minFrameTime = (cfg.maxFps > 0) ? (1.0 / cfg.maxFps) : 0.0;

        cout << "\n=== QUICK STEREO CALIBRATION ===\n";
        cout << "Collecting " << cfg.calibFrames << " good chessboard pairs...\n";
        cout << "Tip: keep the board still for a moment, then move to a new pose.\n";

        if (cfg.show) {
            cv::namedWindow("calib_cam1", cv::WINDOW_NORMAL);
            cv::namedWindow("calib_cam2", cv::WINDOW_NORMAL);
        }

        while ((int)objectPoints.size() < cfg.calibFrames) {

            auto frameStart = std::chrono::high_resolution_clock::now();
            cv::Mat a, b;
            if (!cam1.readBgr(a) || !cam2.readBgr(b)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // Only declare variables once per loop iteration
            BoardDet d1 = detectBoard(a, pattern, prev1.empty() ? nullptr : &prev1);
            BoardDet d2 = detectBoard(b, pattern, prev2.empty() ? nullptr : &prev2);
            bool ok = d1.found && d2.found;
            std::string rejectReason;
            // Accept even if area is small if full set of corners detected
            const bool full1 = (d1.corners.size() == (size_t)pattern.area());
            const bool full2 = (d2.corners.size() == (size_t)pattern.area());
            const bool areaOk1 = (d1.areaRatio >= minAreaRatio) || full1;
            const bool areaOk2 = (d2.areaRatio >= minAreaRatio) || full2;

            if (!d1.found || !d2.found) {
                rejectReason = "Chessboard not found in one or both cameras.";
            } else {
                if (!areaOk1 || !areaOk2)
                    rejectReason += "Area too small: " + std::to_string(d1.areaRatio) + "/" + std::to_string(d2.areaRatio) + ". ";
                if (d1.meanMotionPx > maxMotionPx || d2.meanMotionPx > maxMotionPx)
                    rejectReason += "Motion too high: " + std::to_string(d1.meanMotionPx) + "/" + std::to_string(d2.meanMotionPx) + ". ";
            }
            if (ok) {
                ok = ok && areaOk1 && areaOk2;
                ok = ok && (d1.meanMotionPx <= maxMotionPx) && (d2.meanMotionPx <= maxMotionPx);
            }

            if (d1.found) prev1 = d1.corners;
            if (d2.found) prev2 = d2.corners;

            if (ok) stableCount++; else stableCount = 0;

            if (!ok) {
                // Print diagnostics for rejected frames
                std::cout << "[REJECT] " << rejectReason << " area=" << d1.areaRatio << "/" << d2.areaRatio << " motion=" << d1.meanMotionPx << "/" << d2.meanMotionPx << "\n";
            } else if ((d1.areaRatio < minAreaRatio || d2.areaRatio < minAreaRatio) && (full1 || full2)) {
                // Inform when we accept a small but fully-detected board
                std::cout << "[ACCEPT-SMALL] full corners detected; accepting despite small area (" << d1.areaRatio << "/" << d2.areaRatio << ")\n";
            }

            if (cfg.show) {
                cv::Mat va = a.clone();
                cv::Mat vb = b.clone();
                if (d1.found) cv::drawChessboardCorners(va, pattern, d1.corners, true);
                if (d2.found) cv::drawChessboardCorners(vb, pattern, d2.corners, true);
                {
                    std::ostringstream ss;
                    ss << "pairs=" << objectPoints.size() << "/" << cfg.calibFrames
                       << " stable=" << stableCount
                       << " area=" << std::fixed << std::setprecision(2) << d1.areaRatio << "/" << d2.areaRatio
                       << " mot=" << std::setprecision(2) << d1.meanMotionPx << "/" << d2.meanMotionPx;
                    cv::putText(va, ss.str(), cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                }
                cv::imshow("calib_cam1", va);
                cv::imshow("calib_cam2", vb);
                const int k = cv::waitKey(1);
                if ((k & 0xFF) == 27) throw std::runtime_error("Calibration cancelled (ESC)");
            }

            // Require a few stable frames before accepting.
            if (stableCount >= requiredStable && d1.found && d2.found) {
                objectPoints.push_back(obj);
                img1.push_back(d1.corners);
                img2.push_back(d2.corners);
                stableCount = 0;
                cout << "Captured pair " << objectPoints.size() << "/" << cfg.calibFrames << "\n";
            }

            // Frame rate limiting
            if (minFrameTime > 0.0) {
                auto frameEnd = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration<double>(frameEnd - frameStart).count();
                if (elapsed < minFrameTime) {
                    std::this_thread::sleep_for(std::chrono::milliseconds((int)((minFrameTime - elapsed) * 1000)));
                }
            }
        }

        // Collected enough pairs; run stereo calibration
        cout << "Calibrating stereo from " << objectPoints.size() << " pairs...\n";
        double rms = 0.0;
        cv::Mat E, F;
        try {
            cv::Size imageSize = size1;
            if (imageSize.width == 0 || imageSize.height == 0) imageSize = size2;

            int flags = cv::CALIB_FIX_INTRINSIC;
            flags |= flagsForDistortion(D1);
            flags |= flagsForDistortion(D2);

            rms = cv::stereoCalibrate(objectPoints, img1, img2, K1, D1, K2, D2,
                                      imageSize, R, T, E, F, flags,
                                      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-7));
        } catch (const cv::Exception& e) {
            throw std::runtime_error(std::string("Stereo calibration failed: ") + e.what());
        }

        cout << "Quick stereo calib RMS: " << rms << " px\n";
        if (cfg.baselineMm > 0.0) {
            const double tnorm = cv::norm(T);
            if (tnorm > 1e-9) {
                const double scale = cfg.baselineMm / tnorm;
                T *= scale;
                cout << "Applied baseline constraint: " << cfg.baselineMm << " mm (scale=" << scale << ")\n";
            } else {
                cout << "Baseline constraint requested, but |T| ~ 0; skipped.\n";
            }
        }
        haveStereo = true;
        fisheyeStereo = fisheye;

        if (!cfg.saveStereoExtrinsics.empty()) {
            fs::path out = cfg.saveStereoExtrinsics;
            if (out.is_relative()) out = root / out;
            if (out.parent_path().empty()) out = root / "reports" / out;
            if (!out.parent_path().empty()) fs::create_directories(out.parent_path());
            saveStereoExtrinsicsYml(out, fs::path("(live)"), intr1, intr2, pattern, cfg.squareMm, (int)objectPoints.size(), fisheye, rms, R, T, E, F);
            cout << "Saved stereo extrinsics: " << out.string() << "\n";
        }
        cout << "=== TRACKING ===\n";
    }

    std::ofstream csv;
    if (!cfg.outputCsv.empty()) {
        fs::path out = cfg.outputCsv;
        if (out.is_relative()) out = root / out;
        if (out.parent_path().empty()) out = root / "reports" / out;
        if (!out.parent_path().empty()) fs::create_directories(out.parent_path());
        csv.open(out);
        if (!csv.is_open()) throw std::runtime_error("Failed to open CSV: " + out.string());
        csv << "time,x_mm,y_mm,z_mm,u1,v1,u2,v2,a1,a2\n";
        cout << "Writing CSV: " << fs::absolute(out).string() << "\n";
    }

    if (cfg.show) {
        cv::namedWindow("cam1", cv::WINDOW_NORMAL);
        cv::namedWindow("cam2", cv::WINDOW_NORMAL);
    }

    const int delayMs = (cfg.maxFps > 0) ? (int)std::max(1.0, 1000.0 / (double)cfg.maxFps) : 0;

    int frameIdx = 0;
    BoardDet lastBoard1, lastBoard2;

    cout << "Tracking mode: board\n";

    auto lastFpsTs = std::chrono::high_resolution_clock::now();
    double fpsEma = 0.0;

    while (true) {
        cv::Mat a, b;
        if (!cam1.readBgr(a) || !cam2.readBgr(b)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        BallObs o1;
        BallObs o2;

        const bool doDetect = (cfg.boardSkip <= 0) || (frameIdx % (cfg.boardSkip + 1) == 0);
        if (doDetect) {
            cv::Mat aDet = a;
            cv::Mat bDet = b;
            double scale = cfg.detScale;
            if (scale < 1.0) {
                cv::resize(a, aDet, cv::Size(), scale, scale, cv::INTER_AREA);
                cv::resize(b, bDet, cv::Size(), scale, scale, cv::INTER_AREA);
            }

            lastBoard1 = detectBoard(aDet, cv::Size(cfg.patternCols, cfg.patternRows), nullptr);
            lastBoard2 = detectBoard(bDet, cv::Size(cfg.patternCols, cfg.patternRows), nullptr);

            if (scale < 1.0) {
                for (auto& p : lastBoard1.corners) { p.x /= (float)scale; p.y /= (float)scale; }
                for (auto& p : lastBoard2.corners) { p.x /= (float)scale; p.y /= (float)scale; }
                if (lastBoard1.areaRatio > 0.0) lastBoard1.areaRatio *= scale * scale;
                if (lastBoard2.areaRatio > 0.0) lastBoard2.areaRatio *= scale * scale;
            }
        }

        if (lastBoard1.found) {
            o1.found = true;
            if (!lastBoard1.corners.empty()) {
                cv::Point2f sum(0, 0);
                for (const auto& p : lastBoard1.corners) sum += p;
                o1.centerPx = sum * (1.0f / (float)lastBoard1.corners.size());
            }
            o1.areaPx = lastBoard1.areaRatio * (double)a.cols * (double)a.rows;
        }
        if (lastBoard2.found) {
            o2.found = true;
            if (!lastBoard2.corners.empty()) {
                cv::Point2f sum(0, 0);
                for (const auto& p : lastBoard2.corners) sum += p;
                o2.centerPx = sum * (1.0f / (float)lastBoard2.corners.size());
            }
            o2.areaPx = lastBoard2.areaRatio * (double)b.cols * (double)b.rows;
        }

        cv::Point3d X;
        bool have3d = false;

        if (o1.found && o2.found) {
            cv::Point2f p1n, p2n;
            undistortPointNormalized(o1.centerPx, K1, D1, fisheye, p1n);
            undistortPointNormalized(o2.centerPx, K2, D2, fisheye, p2n);
            have3d = triangulateOne(p1n, p2n, R, T, X);
        }

        if (cfg.show) {
            cv::Mat va = a.clone();
            cv::Mat vb = b.clone();

            if (o1.found) {
                cv::circle(va, o1.centerPx, 6, cv::Scalar(0, 255, 0), 2);
            }
            if (o2.found) {
                cv::circle(vb, o2.centerPx, 6, cv::Scalar(0, 255, 0), 2);
            }

            if (have3d) {
                std::ostringstream ss;
                ss.setf(std::ios::fixed);
                ss << std::setprecision(1);
                ss << "XYZ(mm)= [" << X.x << ", " << X.y << ", " << X.z << "]";
                cv::putText(va, ss.str(), cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);
                std::ostringstream zs;
                zs.setf(std::ios::fixed);
                zs << std::setprecision(1) << "Z=" << X.z << " mm";
                cv::putText(vb, zs.str(), cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);
            } else {
                cv::putText(va, "No stereo 3D (need both views)", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }

            // FPS overlay
            auto now = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration<double>(now - lastFpsTs).count();
            if (dt > 0.0) {
                double instFps = 1.0 / dt;
                fpsEma = (fpsEma <= 0.0) ? instFps : (0.9 * fpsEma + 0.1 * instFps);
            }
            lastFpsTs = now;
            {
                std::ostringstream fs;
                fs.setf(std::ios::fixed);
                fs << std::setprecision(1) << "FPS=" << fpsEma;
                cv::putText(va, fs.str(), cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
            }

            cv::imshow("cam1", va);
            cv::imshow("cam2", vb);

            const int k = cv::waitKey(1);
            if ((k & 0xFF) == 27) break;
        }

        if (csv.is_open()) {
            if (o1.found && o2.found && have3d) {
                csv << nowIso8601() << ','
                    << X.x << ',' << X.y << ',' << X.z << ','
                    << o1.centerPx.x << ',' << o1.centerPx.y << ','
                    << o2.centerPx.x << ',' << o2.centerPx.y << ','
                    << o1.areaPx << ',' << o2.areaPx << "\n";
            }
        }

        if (delayMs > 0) std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
        ++frameIdx;
    }

    if (cfg.show) {
        cv::destroyWindow("cam1");
        cv::destroyWindow("cam2");
    }

    return 0;
}

} // namespace livemotion
