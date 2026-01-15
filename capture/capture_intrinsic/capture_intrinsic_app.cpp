#include "capture_intrinsic_app.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "../mf_camera.h"

namespace capintr {

using std::cerr;
using std::cout;
using std::string;

namespace {

fs::path defaultProjectRoot() {
    const fs::path start = fs::current_path();
    const fs::path cmake = mfcam::findUpwardsForFile(start, "CMakeLists.txt");
    return cmake.empty() ? start : cmake.parent_path();
}

bool isInteger(const string& s) {
    if (s.empty()) return false;
    size_t i = 0;
    if (s[0] == '-' || s[0] == '+') i = 1;
    if (i >= s.size()) return false;
    for (; i < s.size(); ++i) {
        if (s[i] < '0' || s[i] > '9') return false;
    }
    return true;
}

bool parsePattern(const string& s, cv::Size& out) {
    const size_t x = s.find('x');
    if (x == string::npos) return false;
    try {
        const int w = std::stoi(s.substr(0, x));
        const int h = std::stoi(s.substr(x + 1));
        if (w <= 0 || h <= 0) return false;
        out = cv::Size(w, h);
        return true;
    } catch (...) {
        return false;
    }
}

int backendFromString(string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    if (s == "msmf") return cv::CAP_MSMF;
    if (s == "dshow") return cv::CAP_DSHOW;
    return cv::CAP_ANY;
}

int flipCodeFromString(const string& s) {
    if (s == "none") return 999; // sentinel
    if (s == "x") return 0;      // vertical flip
    if (s == "y") return 1;      // horizontal flip
    if (s == "xy") return -1;    // both
    return 999;
}

string nextFlipMode(const string& current) {
    if (current == "none") return "x";
    if (current == "x") return "y";
    if (current == "y") return "xy";
    return "none";
}

void applyRotateFlip(cv::Mat& img, int rotateDeg, int flipCode) {
    if (rotateDeg == 90) {
        cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
    } else if (rotateDeg == 180) {
        cv::rotate(img, img, cv::ROTATE_180);
    } else if (rotateDeg == 270) {
        cv::rotate(img, img, cv::ROTATE_90_COUNTERCLOCKWISE);
    }

    if (flipCode != 999) {
        cv::flip(img, img, flipCode);
    }
}

double blurScoreVarianceOfLaplacian(const cv::Mat& gray) {
    cv::Mat lap;
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev[0] * stddev[0];
}

double meanCornerShift(const std::optional<std::vector<cv::Point2f>>& prev, const std::vector<cv::Point2f>& now) {
    if (!prev.has_value()) return 1e9;
    if (prev->size() != now.size() || now.empty()) return 1e9;

    double sum = 0.0;
    for (size_t i = 0; i < now.size(); ++i) {
        sum += cv::norm((*prev)[i] - now[i]);
    }
    return sum / (double)now.size();
}

double boardNeighborSizePx(const std::vector<cv::Point2f>& corners, const cv::Size& pattern) {
    if ((int)corners.size() != pattern.area()) return 0.0;

    const int cols = pattern.width;
    const int rows = pattern.height;
    double sumD = 0.0;
    int cnt = 0;
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            const int idx = y * cols + x;
            if (x + 1 < cols) {
                sumD += cv::norm(corners[idx] - corners[idx + 1]);
                ++cnt;
            }
            if (y + 1 < rows) {
                sumD += cv::norm(corners[idx] - corners[idx + cols]);
                ++cnt;
            }
        }
    }
    return (cnt > 0) ? (sumD / (double)cnt) : 0.0;
}

double minBorderPx(const std::vector<cv::Point2f>& corners, int w, int h) {
    double m = 1e18;
    for (const auto& p : corners) {
        m = std::min(m, (double)p.x);
        m = std::min(m, (double)p.y);
        m = std::min(m, (double)(w - 1) - (double)p.x);
        m = std::min(m, (double)(h - 1) - (double)p.y);
    }
    return (m == 1e18) ? 0.0 : m;
}

double boardAreaRatioBbox(const std::vector<cv::Point2f>& corners, int w, int h) {
    if (corners.empty() || w <= 0 || h <= 0) return 0.0;
    const cv::Rect bbox = cv::boundingRect(corners);
    const double imgArea = (double)w * (double)h;
    if (imgArea <= 0.0) return 0.0;
    return (double)bbox.area() / imgArea;
}

cv::Point2d centroidOfCorners(const std::vector<cv::Point2f>& corners) {
    if (corners.empty()) return {0.0, 0.0};
    double sx = 0.0;
    double sy = 0.0;
    for (const auto& p : corners) {
        sx += p.x;
        sy += p.y;
    }
    return {sx / (double)corners.size(), sy / (double)corners.size()};
}

double centroidShiftNorm(const std::optional<std::vector<cv::Point2f>>& prev,
                         const std::vector<cv::Point2f>& now,
                         int w,
                         int h) {
    if (!prev.has_value() || prev->empty() || now.empty() || w <= 0 || h <= 0) return 1e9;
    const cv::Point2d a = centroidOfCorners(*prev);
    const cv::Point2d b = centroidOfCorners(now);
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double d = std::sqrt(dx * dx + dy * dy);
    const double diag = std::sqrt((double)w * (double)w + (double)h * (double)h);
    return (diag > 0.0) ? (d / diag) : 1e9;
}

struct GateResult {
    bool found = false;
    bool blurOk = false;
    bool shiftOk = false;
    bool borderOk = false;
    bool boardOk = false;
    bool areaOk = false;
    bool centerOk = false;

    double blur = 0.0;
    double shift = 0.0;
    double border = 0.0;
    double board = 0.0;
    double areaRatio = 0.0;
    double centerShift = 0.0;

    bool good() const { return found && blurOk && shiftOk && borderOk && boardOk && areaOk && centerOk; }
};

GateResult evaluateGates(const AppConfig& cfg, const cv::Mat& gray, const cv::Mat& frameBgr, const cv::Size& pattern,
                         const std::optional<std::vector<cv::Point2f>>& prevSavedCorners, std::vector<cv::Point2f>& inOutCorners,
                         cv::Mat* drawOnBgr) {
    GateResult r;

    r.blur = blurScoreVarianceOfLaplacian(gray);

    // Prefer SB (robust/accurate), fallback to classic.
    r.found = cv::findChessboardCornersSB(gray, pattern, inOutCorners, cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ACCURACY);
    if (!r.found) {
        r.found = cv::findChessboardCorners(gray, pattern, inOutCorners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    }
    if (!r.found) return r;

    cv::cornerSubPix(gray, inOutCorners, cv::Size(15, 15), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 80, 1e-12));

    if (drawOnBgr) {
        cv::drawChessboardCorners(*drawOnBgr, pattern, inOutCorners, true);
    }

    r.shift = meanCornerShift(prevSavedCorners, inOutCorners);
    r.border = minBorderPx(inOutCorners, frameBgr.cols, frameBgr.rows);
    r.board = boardNeighborSizePx(inOutCorners, pattern);
    r.areaRatio = boardAreaRatioBbox(inOutCorners, frameBgr.cols, frameBgr.rows);
    r.centerShift = centroidShiftNorm(prevSavedCorners, inOutCorners, frameBgr.cols, frameBgr.rows);

    r.blurOk = (r.blur >= cfg.minBlur);
    r.shiftOk = (!prevSavedCorners.has_value()) ? true : (r.shift >= cfg.minShiftPx);
    r.borderOk = (r.border >= cfg.minBorderPx);
    r.boardOk = (r.board >= cfg.minBoardPx);
    r.areaOk = (r.areaRatio >= cfg.minBoardAreaRatio);
    r.centerOk = (!prevSavedCorners.has_value()) ? true : (r.centerShift >= cfg.minCenterShiftNorm);
    return r;
}

} // namespace

void printUsage(std::ostream& os) {
    os << "capture_intrinsic (single camera)\n"
          "Captures GOOD chessboard images for subpixel-accurate intrinsics.\n\n"
          "Usage:\n"
          "  capture_intrinsic.exe --cam cam3 --count 60 --pattern 8x5\n"
          "  capture_intrinsic.exe --cams-yml cameras.yml --cam cam1 --out data/calib_cam1 --count 60 --pattern 8x5\n"
          "  capture_intrinsic.exe --symbolic-link \\\"\\\\?\\usb#...\\\" --out data/calib_camX --count 60 --pattern 8x5\n"
          "  capture_intrinsic.exe --camera 0 --backend msmf --out data/calib_cam1 --count 60 --pattern 8x5  (fallback)\n\n"
          "Options:\n"
          "  --cam <camKey>             Camera key in cameras.yml, e.g. cam1/cam2/cam3 (recommended)\n"
          "  --cameras-yml <path>       Path to cameras.yml (optional; auto-found if omitted)\n"
          "  --symbolic-link <string>   Open camera by Media Foundation symbolic link\n"
          "  --camera <int|camKey>      If numeric: OpenCV index fallback. If camKey: same as --cam\n"
          "  --out <path>               Output dir (default data/calib_<camKey> or data/calib_cam<index+1>)\n"
          "  --count <int>              Images to save (default 60)\n"
          "  --pattern <WxH>            Inner corners (default 8x5)\n"
          "  --backend <any|msmf|dshow> OpenCV backend hint (default any)\n"
          "  --width <int>              Request width (MF mode)\n"
          "  --height <int>             Request height (MF mode)\n"
          "  --fps <int>                Request fps (MF mode)\n"
          "  --interval-ms <int>        Auto-capture min interval (default 700)\n"
          "  --auto <0|1>               Auto capture (default 1)\n"
          "  --rotate <0|90|180|270>    Rotate frames before detect/save (default 0)\n"
          "  --flip <none|x|y|xy>       Flip frames before detect/save (default none)\n"
          "  --min-blur <float>         Variance-of-Laplacian threshold (default 140)\n"
          "  --min-shift-px <float>     Mean corner shift since last save (default 14)\n"
          "  --min-border-px <float>    Min corner distance to image border (default 20)\n"
          "  --min-board-px <float>     Min board neighbor distance (default 20)\n"
          "  --min-board-area <float>   Min board bbox area ratio (default 0.10)\n"
          "  --min-center-shift <float> Min centroid shift (normalized by image diagonal) (default 0.05)\n"
          "  --stable-frames <int>      Require N stable frames before save (default 4)\n"
          "  --max-jitter-px <float>    Max mean corner jitter between frames (default 1.8)\n"
          "  --png-compression <0-9>    PNG compression (lossless). 0 fastest (default 0)\n\n"
          "Runtime:\n"
          "  [Space]=save  [A]=toggle auto  [R]=rotate  [F]=flip  [S]=save raw (debug)  [Esc]=quit\n";
}

bool parseArgs(int argc, char** argv, AppConfig& cfg, string& errorMessage) {
    errorMessage.clear();

    auto need = [&](int& i, const char* name) -> string {
        if (i + 1 >= argc) {
            std::ostringstream ss;
            ss << "Missing value for " << name;
            throw std::runtime_error(ss.str());
        }
        return argv[++i];
    };

    try {
        for (int i = 1; i < argc; ++i) {
            const string a = argv[i];
            if (a == "--help" || a == "-h") {
                errorMessage = "help";
                return false;
            }

            if (a == "--camera") {
                const string v = need(i, "--camera");
                if (isInteger(v)) {
                    cfg.cameraIndex = std::stoi(v);
                } else {
                    cfg.camKey = v;
                }
                continue;
            }
            if (a == "--cam") {
                cfg.camKey = need(i, "--cam");
                continue;
            }
            if (a == "--cameras-yml" || a == "--cams-yml") {
                cfg.camerasYmlPath = need(i, "--cameras-yml");
                continue;
            }
            if (a == "--symbolic-link") {
                cfg.symbolicLink = need(i, "--symbolic-link");
                continue;
            }
            if (a == "--out") {
                cfg.outPath = need(i, "--out");
                continue;
            }
            if (a == "--count") {
                cfg.count = std::stoi(need(i, "--count"));
                continue;
            }
            if (a == "--pattern") {
                cv::Size p;
                if (!parsePattern(need(i, "--pattern"), p)) {
                    errorMessage = "Bad --pattern; expected WxH like 8x5";
                    return false;
                }
                cfg.pattern = p;
                continue;
            }
            if (a == "--backend") {
                cfg.backend = need(i, "--backend");
                continue;
            }
            if (a == "--width") {
                cfg.reqWidth = (unsigned)std::stoul(need(i, "--width"));
                continue;
            }
            if (a == "--height") {
                cfg.reqHeight = (unsigned)std::stoul(need(i, "--height"));
                continue;
            }
            if (a == "--fps") {
                cfg.reqFps = (unsigned)std::stoul(need(i, "--fps"));
                continue;
            }
            if (a == "--interval-ms") {
                cfg.intervalMs = std::stoi(need(i, "--interval-ms"));
                continue;
            }
            if (a == "--auto") {
                cfg.autoEnabled = (std::stoi(need(i, "--auto")) != 0);
                continue;
            }
            if (a == "--rotate") {
                cfg.rotateDeg = std::stoi(need(i, "--rotate"));
                if (!(cfg.rotateDeg == 0 || cfg.rotateDeg == 90 || cfg.rotateDeg == 180 || cfg.rotateDeg == 270)) {
                    errorMessage = "--rotate must be one of 0,90,180,270";
                    return false;
                }
                continue;
            }
            if (a == "--flip") {
                cfg.flipMode = need(i, "--flip");
                std::transform(cfg.flipMode.begin(), cfg.flipMode.end(), cfg.flipMode.begin(), [](unsigned char c) { return (char)std::tolower(c); });
                if (flipCodeFromString(cfg.flipMode) == 999 && cfg.flipMode != "none") {
                    errorMessage = "--flip must be one of none,x,y,xy";
                    return false;
                }
                continue;
            }
            if (a == "--min-blur") {
                cfg.minBlur = std::stod(need(i, "--min-blur"));
                continue;
            }
            if (a == "--min-shift-px") {
                cfg.minShiftPx = std::stod(need(i, "--min-shift-px"));
                continue;
            }
            if (a == "--min-border-px") {
                cfg.minBorderPx = std::stod(need(i, "--min-border-px"));
                continue;
            }
            if (a == "--min-board-px") {
                cfg.minBoardPx = std::stod(need(i, "--min-board-px"));
                continue;
            }
            if (a == "--min-board-area" || a == "--min-board-area-ratio") {
                cfg.minBoardAreaRatio = std::stod(need(i, "--min-board-area"));
                continue;
            }
            if (a == "--min-center-shift" || a == "--min-center-shift-norm") {
                cfg.minCenterShiftNorm = std::stod(need(i, "--min-center-shift"));
                continue;
            }
            if (a == "--stable-frames") {
                cfg.stableFrames = std::max(0, std::stoi(need(i, "--stable-frames")));
                continue;
            }
            if (a == "--max-jitter-px") {
                cfg.maxJitterPx = std::stod(need(i, "--max-jitter-px"));
                continue;
            }
            if (a == "--png-compression") {
                cfg.pngCompression = std::clamp(std::stoi(need(i, "--png-compression")), 0, 9);
                continue;
            }

            errorMessage = "Unknown arg: " + a;
            return false;
        }
    } catch (const std::exception& e) {
        errorMessage = e.what();
        return false;
    }

    if (!cfg.camKey.empty() && !cfg.symbolicLink.empty()) {
        errorMessage = "Use only one of --cam or --symbolic-link";
        return false;
    }
    if (cfg.count <= 0) {
        errorMessage = "--count must be > 0";
        return false;
    }
    return true;
}

bool resolvePathsAndCamera(AppConfig& cfg, ResolvedPaths& out, string& errorMessage) {
    errorMessage.clear();
    out = {};

    if (!cfg.camerasYmlPath.empty()) {
        out.camerasYml = fs::absolute(fs::path(cfg.camerasYmlPath));
        out.projectRoot = out.camerasYml.parent_path();
    } else {
        out.projectRoot = defaultProjectRoot();
        out.camerasYml = mfcam::findUpwardsForFile(fs::current_path(), "cameras.yml");
    }

    if (cfg.symbolicLink.empty() && !cfg.camKey.empty()) {
        if (out.camerasYml.empty()) {
            errorMessage = "cameras.yml not found. Provide --cameras-yml <path>";
            return false;
        }
        if (!mfcam::loadSymbolicLinkFromCamerasYml(out.camerasYml, cfg.camKey, cfg.symbolicLink)) {
            std::ostringstream ss;
            ss << "Could not find '" << cfg.camKey << "'->symbolic_link in " << out.camerasYml.string();
            errorMessage = ss.str();
            return false;
        }
    }

    const bool useMf = !cfg.symbolicLink.empty();
    if (!useMf && cfg.cameraIndex < 0) {
        errorMessage = "Provide --cam camX (recommended) or --symbolic-link, or fallback --camera <index>";
        return false;
    }

    if (!cfg.outPath.empty()) {
        fs::path p(cfg.outPath);
        out.outDir = p.is_absolute() ? p : (out.projectRoot / p);
    } else if (!cfg.camKey.empty()) {
        out.outDir = out.projectRoot / "data" / ("calib_" + cfg.camKey);
    } else {
        std::ostringstream ss;
        ss << "calib_cam" << (cfg.cameraIndex + 1);
        out.outDir = out.projectRoot / "data" / ss.str();
    }
    fs::create_directories(out.outDir);
    return true;
}

int run(const AppConfig& cfg) {
    ResolvedPaths paths;
    string err;
    AppConfig cfgMut = cfg; // resolvePaths may fill symbolicLink
    if (!resolvePathsAndCamera(cfgMut, paths, err)) {
        cerr << "ERROR: " << err << "\n";
        return 2;
    }

    const bool useMf = !cfgMut.symbolicLink.empty();
    cv::VideoCapture cap;
    mfcam::Camera mf;

    if (useMf) {
        mfcam::OpenOptions opts;
        opts.width = cfgMut.reqWidth;
        opts.height = cfgMut.reqHeight;
        opts.fps = cfgMut.reqFps;
        if (!mf.openSymbolicLink(cfgMut.symbolicLink, opts)) {
            cerr << "ERROR: Cannot open camera by symbolic link\n";
            if (!cfgMut.camKey.empty()) cerr << "  cam: " << cfgMut.camKey << "\n";
            cerr << "  symbolic_link: " << cfgMut.symbolicLink << "\n";
            return 1;
        }
    } else {
        const int capBackend = backendFromString(cfgMut.backend);
        if (capBackend == cv::CAP_ANY) {
            cap.open(cfgMut.cameraIndex);
        } else {
            cap.open(cfgMut.cameraIndex, capBackend);
        }
        if (!cap.isOpened()) {
            cerr << "ERROR: Cannot open camera " << cfgMut.cameraIndex << " (OpenCV backend='" << cfgMut.backend << "')\n";
            cerr << "Hint: Prefer '--cam camX' so we open via cameras.yml / Media Foundation.\n";
            return 1;
        }
    }

    cout << "capture_intrinsic\n";
    cout << "Camera: "
         << (useMf ? (cfgMut.camKey.empty() ? string("(symbolic-link)") : cfgMut.camKey)
                   : (string("index ") + std::to_string(cfgMut.cameraIndex) + " (OpenCV)"))
         << " | Pattern: " << cfgMut.pattern.width << "x" << cfgMut.pattern.height << "\n";
    cout << "Output: " << paths.outDir.string() << " | Target: " << cfgMut.count << "\n";
    cout << "Auto: " << (cfgMut.autoEnabled ? "ON" : "OFF") << " | interval_ms: " << cfgMut.intervalMs << "\n";
        cout << "Transform: rotate=" << cfgMut.rotateDeg << " flip=" << cfgMut.flipMode << "\n";
    cout << "Gates: min_blur=" << cfgMut.minBlur << " min_shift_px=" << cfgMut.minShiftPx << " min_border_px=" << cfgMut.minBorderPx
            << " min_board_px=" << cfgMut.minBoardPx
            << " min_board_area=" << cfgMut.minBoardAreaRatio
                << " min_center_shift=" << cfgMut.minCenterShiftNorm
                << " stable_frames=" << cfgMut.stableFrames
                << " max_jitter_px=" << cfgMut.maxJitterPx << "\n";
        cout << "Controls: [Space]=save  [A]=auto  [R]=rotate  [F]=flip  [S]=raw  [Esc]=quit\n\n";

    const string win = "capture_intrinsic";
    cv::namedWindow(win, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);

    int saved = 0;
    int rawSaved = 0;
    std::optional<std::vector<cv::Point2f>> prevSavedCorners;
    std::optional<std::vector<cv::Point2f>> prevFrameCorners;
    int stableStreak = 0;
    double lastJitter = 0.0;
    auto lastSave = std::chrono::steady_clock::now() - std::chrono::milliseconds(std::max(cfgMut.intervalMs, 1));
    auto lastAutoSkipLog = std::chrono::steady_clock::now() - std::chrono::milliseconds(2000);

    while (saved < cfgMut.count) {
        cv::Mat frame;
        if (useMf) {
            if (!mf.readBgr(frame) || frame.empty()) continue;
        } else {
            if (!cap.read(frame) || frame.empty()) continue;
        }

        if (cfgMut.rotateDeg != 0 || cfgMut.flipMode != "none") {
            applyRotateFlip(frame, cfgMut.rotateDeg, flipCodeFromString(cfgMut.flipMode));
        }

        cv::Mat frameRaw = frame.clone();
        cv::Mat gray;
        if (frame.channels() == 3) {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        } else if (frame.channels() == 4) {
            cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
        } else {
            gray = frame;
        }

        std::vector<cv::Point2f> corners;
        GateResult g = evaluateGates(cfgMut, gray, frame, cfgMut.pattern, prevSavedCorners, corners, &frame);

        // Anti-wobble tracking: measure jitter between consecutive frames (not just between saves).
        if (g.found) {
            lastJitter = meanCornerShift(prevFrameCorners, corners);
            const bool jitterOk = (!prevFrameCorners.has_value()) ? true : (lastJitter <= cfgMut.maxJitterPx);
            stableStreak = jitterOk ? (stableStreak + 1) : 0;
            prevFrameCorners = corners;
        } else {
            stableStreak = 0;
            prevFrameCorners.reset();
            lastJitter = 0.0;
        }

        {
            std::ostringstream ss;
            ss << saved << "/" << cfgMut.count << "  auto=" << (cfgMut.autoEnabled ? 1 : 0) << "  "
               << (g.found ? (g.good() ? "GOOD" : "FOUND") : "NO") << "  blur=" << (int)g.blur;
            if (g.found) {
                ss << "  shift=" << std::fixed << std::setprecision(1) << (prevSavedCorners.has_value() ? g.shift : 0.0);
                ss << "  border=" << std::fixed << std::setprecision(1) << g.border;
                ss << "  size=" << std::fixed << std::setprecision(1) << g.board;
                ss << "  area=" << std::fixed << std::setprecision(3) << g.areaRatio;
                ss << "  cshift=" << std::fixed << std::setprecision(3) << (prevSavedCorners.has_value() ? g.centerShift : 0.0);
                ss << "  jitter=" << std::fixed << std::setprecision(2) << (prevFrameCorners.has_value() ? lastJitter : 0.0);
                ss << "  stable=" << stableStreak << "/" << cfgMut.stableFrames;
            }

            const cv::Scalar col = g.good() ? cv::Scalar(0, 255, 0) : (g.found ? cv::Scalar(0, 200, 255) : cv::Scalar(0, 0, 255));
            cv::putText(frame, ss.str(), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 3);
            cv::putText(frame, ss.str(), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, col, 1);

            if (!g.found) {
                cv::putText(frame, "Searching chessboard...", {10, 70}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }
        }

        cv::imshow(win, frame);

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 27) break;

        if (key == 'a' || key == 'A') {
            cfgMut.autoEnabled = !cfgMut.autoEnabled;
            cout << "Auto: " << (cfgMut.autoEnabled ? "ON" : "OFF") << "\n";
        }

        if (key == 'r' || key == 'R') {
            cfgMut.rotateDeg = (cfgMut.rotateDeg + 90) % 360;
            if (!(cfgMut.rotateDeg == 0 || cfgMut.rotateDeg == 90 || cfgMut.rotateDeg == 180 || cfgMut.rotateDeg == 270)) {
                cfgMut.rotateDeg = 0;
            }
            cout << "Rotate: " << cfgMut.rotateDeg << "\n";
        }

        if (key == 'f' || key == 'F') {
            cfgMut.flipMode = nextFlipMode(cfgMut.flipMode);
            cout << "Flip: " << cfgMut.flipMode << "\n";
        }

        if (key == 's' || key == 'S') {
            const fs::path outPath = paths.outDir / ("raw_" + cv::format("%04d", rawSaved) + ".png");
            const std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, cfgMut.pngCompression};
            if (cv::imwrite(outPath.string(), frameRaw, params)) {
                ++rawSaved;
                cout << "Saved raw: " << outPath.string() << "\n";
            }
        }

        const auto now = std::chrono::steady_clock::now();
        const bool manualSave = (key == 32);
        const bool autoSave = cfgMut.autoEnabled
                              && (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSave).count() >= cfgMut.intervalMs);

        if ((manualSave || autoSave) && g.found) {
            const bool shouldLog = manualSave
                                   || (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastAutoSkipLog).count() >= 1200);

            auto skip = [&](const string& msg) {
                if (shouldLog) cout << "Skip: " << msg << (autoSave ? " (auto)" : "") << "\n";
                lastAutoSkipLog = now;
            };

            if (!g.blurOk) {
                std::ostringstream ss;
                ss << "blur " << g.blur << " < " << cfgMut.minBlur;
                skip(ss.str());
                continue;
            }
            if (!g.boardOk) {
                std::ostringstream ss;
                ss << "board too small (min_board_px=" << cfgMut.minBoardPx << ")";
                skip(ss.str());
                continue;
            }
            if (!g.areaOk) {
                std::ostringstream ss;
                ss << "board bbox too small (min_board_area=" << cfgMut.minBoardAreaRatio << ")";
                skip(ss.str());
                continue;
            }
            if (!g.borderOk) {
                std::ostringstream ss;
                ss << "too close to border (min_border_px=" << cfgMut.minBorderPx << ")";
                skip(ss.str());
                continue;
            }
            if (!g.shiftOk) {
                std::ostringstream ss;
                ss << "pose too similar (min_shift_px=" << cfgMut.minShiftPx << ")";
                skip(ss.str());
                continue;
            }
            if (!g.centerOk) {
                std::ostringstream ss;
                ss << "center too similar (min_center_shift=" << cfgMut.minCenterShiftNorm << ")";
                skip(ss.str());
                continue;
            }

            if (cfgMut.stableFrames > 0) {
                if (stableStreak < cfgMut.stableFrames) {
                    std::ostringstream ss;
                    ss << "wobbly (need stable " << cfgMut.stableFrames << " frames, got " << stableStreak
                       << "; max_jitter_px=" << cfgMut.maxJitterPx << ")";
                    skip(ss.str());
                    continue;
                }
            }

            const fs::path outPath = paths.outDir / ("calib_" + cv::format("%04d", saved) + ".png");
            const std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, cfgMut.pngCompression};
            if (!cv::imwrite(outPath.string(), frameRaw, params)) {
                cerr << "ERROR: Failed to write " << outPath.string() << "\n";
                continue;
            }

            prevSavedCorners = corners;
            lastSave = now;
            ++saved;
            cout << (autoSave ? "Auto-saved: " : "Saved: ") << outPath.string() << "\n";
        } else if (manualSave && !g.found) {
            cout << "Skip: chessboard NOT found\n";
        }
    }

    if (!useMf) cap.release();
    cv::destroyAllWindows();
    cout << "Done. Saved " << saved << " images to " << paths.outDir.string() << "\n";
    return 0;
}

} // namespace capintr
