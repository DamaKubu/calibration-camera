#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "../mf_camera.h"

namespace fs = std::filesystem;

static fs::path findUpwardsForFile(fs::path startDir, const fs::path& filename, int maxHops = 8) {
    fs::path cur = fs::absolute(startDir);
    for (int hop = 0; hop <= maxHops; ++hop) {
        fs::path candidate = cur / filename;
        if (fs::exists(candidate)) return candidate;
        if (!cur.has_parent_path()) break;
        fs::path parent = cur.parent_path();
        if (parent == cur) break;
        cur = parent;
    }
    return {};
}

static fs::path defaultProjectRoot() {
    fs::path start = fs::current_path();
    fs::path cmake = findUpwardsForFile(start, "CMakeLists.txt");
    if (!cmake.empty()) return cmake.parent_path();
    return start;
}

static bool isNumber(const std::string& s) {
    if (s.empty()) return false;
    size_t i = 0;
    if (s[0] == '-' || s[0] == '+') i = 1;
    if (i >= s.size()) return false;
    for (; i < s.size(); ++i) {
        if (s[i] < '0' || s[i] > '9') return false;
    }
    return true;
}

static void printUsage() {
    std::cout
        << "capture_intrinsic (single camera)\n"
        << "Captures GOOD chessboard images for subpixel-accurate intrinsics.\n\n"
        << "Usage:\n"
    << "  capture_intrinsic.exe --cam cam3 --count 60 --pattern 8x5\n"
    << "  capture_intrinsic.exe --cams-yml cameras.yml --cam cam1 --out data/calib_cam1 --count 60 --pattern 8x5\n"
    << "  capture_intrinsic.exe --symbolic-link \"\\\\?\\usb#...\" --out data/calib_camX --count 60 --pattern 8x5\n"
    << "  capture_intrinsic.exe --camera 0 --backend msmf --out data/calib_cam1 --count 60 --pattern 8x5  (fallback)\n\n"
        << "Options:\n"
    << "  --cam <camKey>            Camera key in cameras.yml, e.g. cam1/cam2/cam3 (recommended)\n"
    << "  --cameras-yml <path>      Path to cameras.yml (optional; auto-found if omitted)\n"
    << "  --symbolic-link <string>  Open camera by Media Foundation symbolic link\n"
    << "  --camera <int|camKey>     If numeric: OpenCV index fallback. If camKey: same as --cam\n"
    << "  --out <path>              Output dir (default data/calib_<camKey> or data/calib_cam<index+1>)\n"
        << "  --count <int>             Images to save (default 60)\n"
        << "  --pattern <WxH>           Inner corners (default 8x5)\n"
        << "  --backend <any|msmf|dshow> OpenCV backend hint (default any)\n"
    << "  --width <int>             Request width (MF mode)\n"
    << "  --height <int>            Request height (MF mode)\n"
    << "  --fps <int>               Request fps (MF mode)\n"
        << "  --interval-ms <int>       Auto-capture min interval (default 700)\n"
        << "  --auto <0|1>              Auto capture (default 1)\n"
        << "  --min-blur <float>        Variance-of-Laplacian threshold (default 140)\n"
        << "  --min-shift-px <float>    Mean corner shift since last save (default 14)\n"
        << "  --min-border-px <float>   Min corner distance to image border (default 20)\n"
        << "  --min-board-px <float>    Min board neighbor distance (default 20)\n"
        << "  --png-compression <0-9>   PNG compression (lossless). 0 fastest (default 0)\n\n"
        << "Runtime:\n"
        << "  [Space]=save  [A]=toggle auto  [S]=save raw (debug)  [Esc]=quit\n";
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

static int backendFromString(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    if (s == "msmf") return cv::CAP_MSMF;
    if (s == "dshow") return cv::CAP_DSHOW;
    return cv::CAP_ANY;
}

static double blurScoreVarianceOfLaplacian(const cv::Mat& gray) {
    cv::Mat lap;
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev[0] * stddev[0];
}

static double meanCornerShift(const std::optional<std::vector<cv::Point2f>>& prev, const std::vector<cv::Point2f>& now) {
    if (!prev.has_value()) return 1e9;
    if (prev->size() != now.size() || now.empty()) return 1e9;

    double sum = 0.0;
    for (size_t i = 0; i < now.size(); ++i) {
        sum += cv::norm((*prev)[i] - now[i]);
    }
    return sum / (double)now.size();
}

static double boardNeighborSizePx(const std::vector<cv::Point2f>& corners, const cv::Size& pattern) {
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
                cnt++;
            }
            if (y + 1 < rows) {
                sumD += cv::norm(corners[idx] - corners[idx + cols]);
                cnt++;
            }
        }
    }
    return (cnt > 0) ? (sumD / (double)cnt) : 0.0;
}

static double minBorderPx(const std::vector<cv::Point2f>& corners, int w, int h) {
    double m = 1e18;
    for (const auto& p : corners) {
        m = std::min(m, (double)p.x);
        m = std::min(m, (double)p.y);
        m = std::min(m, (double)(w - 1) - (double)p.x);
        m = std::min(m, (double)(h - 1) - (double)p.y);
    }
    if (m == 1e18) return 0.0;
    return m;
}

int main(int argc, char** argv) {
    int cameraIndex = -1;
    std::string camKey;
    std::string camerasYml;
    std::string symbolicLink;

    std::string outArg;
    int count = 60;
    cv::Size pattern(8, 5);
    std::string backend = "any";

    unsigned reqWidth = 0;
    unsigned reqHeight = 0;
    unsigned reqFps = 0;

    bool autoEnabled = true;
    int intervalMs = 700;

    double minBlur = 140.0;
    double minShiftPx = 14.0;
    double minBorder = 20.0;
    double minBoard = 20.0;

    int pngCompression = 0;

    auto need = [&](int& i, const char* name) -> std::string {
        if (i + 1 >= argc) {
            std::cerr << "Missing value for " << name << "\n";
            std::exit(2);
        }
        return argv[++i];
    };

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            printUsage();
            return 0;
        } else if (a == "--camera") {
            const std::string v = need(i, "--camera");
            if (isNumber(v)) {
                cameraIndex = std::stoi(v);
            } else {
                camKey = v;
            }
        } else if (a == "--cam") {
            camKey = need(i, "--cam");
        } else if (a == "--cameras-yml" || a == "--cams-yml") {
            camerasYml = need(i, "--cameras-yml");
        } else if (a == "--symbolic-link") {
            symbolicLink = need(i, "--symbolic-link");
        } else if (a == "--out") {
            outArg = need(i, "--out");
        } else if (a == "--count") {
            count = std::stoi(need(i, "--count"));
        } else if (a == "--pattern") {
            cv::Size p;
            if (!parsePattern(need(i, "--pattern"), p)) {
                std::cerr << "Bad --pattern; expected WxH like 8x5\n";
                return 2;
            }
            pattern = p;
        } else if (a == "--backend") {
            backend = need(i, "--backend");
        } else if (a == "--width") {
            reqWidth = (unsigned)std::stoul(need(i, "--width"));
        } else if (a == "--height") {
            reqHeight = (unsigned)std::stoul(need(i, "--height"));
        } else if (a == "--fps") {
            reqFps = (unsigned)std::stoul(need(i, "--fps"));
        } else if (a == "--interval-ms") {
            intervalMs = std::stoi(need(i, "--interval-ms"));
        } else if (a == "--auto") {
            autoEnabled = (std::stoi(need(i, "--auto")) != 0);
        } else if (a == "--min-blur") {
            minBlur = std::stod(need(i, "--min-blur"));
        } else if (a == "--min-shift-px") {
            minShiftPx = std::stod(need(i, "--min-shift-px"));
        } else if (a == "--min-border-px") {
            minBorder = std::stod(need(i, "--min-border-px"));
        } else if (a == "--min-board-px") {
            minBoard = std::stod(need(i, "--min-board-px"));
        } else if (a == "--png-compression") {
            pngCompression = std::clamp(std::stoi(need(i, "--png-compression")), 0, 9);
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            printUsage();
            return 2;
        }
    }

    if (!camKey.empty() && !symbolicLink.empty()) {
        std::cerr << "ERROR: Use only one of --cam or --symbolic-link\n";
        return 2;
    }
    if (count <= 0) {
        std::cerr << "ERROR: --count must be > 0\n";
        return 2;
    }

    fs::path projectRoot;
    fs::path ymlPath;
    if (!camerasYml.empty()) {
        ymlPath = fs::absolute(fs::path(camerasYml));
        projectRoot = ymlPath.parent_path();
    } else {
        projectRoot = defaultProjectRoot();
        ymlPath = mfcam::findUpwardsForFile(fs::current_path(), "cameras.yml");
    }

    if (symbolicLink.empty() && !camKey.empty()) {
        if (ymlPath.empty()) {
            std::cerr << "ERROR: cameras.yml not found. Provide --cameras-yml <path>\n";
            return 2;
        }
        if (!mfcam::loadSymbolicLinkFromCamerasYml(ymlPath, camKey, symbolicLink)) {
            std::cerr << "ERROR: Could not find '" << camKey << "'->symbolic_link in " << ymlPath.string() << "\n";
            return 2;
        }
    }

    const bool useMf = !symbolicLink.empty();
    if (!useMf && cameraIndex < 0) {
        std::cerr << "ERROR: Provide a camera using --cam camX (recommended) or --symbolic-link, or fallback --camera <index>\n";
        return 2;
    }
    fs::path outDir;
    if (!outArg.empty()) {
        fs::path p(outArg);
        outDir = p.is_absolute() ? p : (projectRoot / p);
    } else {
        if (!camKey.empty()) {
            outDir = projectRoot / "data" / ("calib_" + camKey);
        } else {
            std::ostringstream ss;
            ss << "calib_cam" << (cameraIndex + 1);
            outDir = projectRoot / "data" / ss.str();
        }
    }
    fs::create_directories(outDir);

    cv::VideoCapture cap;
    mfcam::Camera mf;
    if (useMf) {
        mfcam::OpenOptions opts;
        opts.width = reqWidth;
        opts.height = reqHeight;
        opts.fps = reqFps;
        if (!mf.openSymbolicLink(symbolicLink, opts)) {
            std::cerr << "ERROR: Cannot open camera by symbolic link\n";
            if (!camKey.empty()) std::cerr << "  cam: " << camKey << "\n";
            std::cerr << "  symbolic_link: " << symbolicLink << "\n";
            return 1;
        }
    } else {
        const int capBackend = backendFromString(backend);
        if (capBackend == cv::CAP_ANY) {
            cap.open(cameraIndex);
        } else {
            cap.open(cameraIndex, capBackend);
        }
        if (!cap.isOpened()) {
            std::cerr << "ERROR: Cannot open camera " << cameraIndex << " (OpenCV backend='" << backend << "')\n";
            std::cerr << "Hint: Prefer '--cam camX' so we open via cameras.yml / Media Foundation.\n";
            return 1;
        }
    }

    std::cout << "capture_intrinsic\n";
    if (useMf) {
        std::cout << "Camera: " << (camKey.empty() ? std::string("(symbolic-link)") : camKey) << " | Pattern: " << pattern.width << "x" << pattern.height << "\n";
    } else {
        std::cout << "Camera: index " << cameraIndex << " (OpenCV) | Pattern: " << pattern.width << "x" << pattern.height << "\n";
    }
    std::cout << "Output: " << outDir.string() << " | Target: " << count << "\n";
    std::cout << "Auto: " << (autoEnabled ? "ON" : "OFF") << " | interval_ms: " << intervalMs << "\n";
    std::cout << "Gates: min_blur=" << minBlur << " min_shift_px=" << minShiftPx << " min_border_px=" << minBorder << " min_board_px=" << minBoard << "\n";
    std::cout << "Controls: [Space]=save  [A]=auto  [S]=raw  [Esc]=quit\n\n";

    const std::string win = "capture_intrinsic";
    cv::namedWindow(win, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);

    int saved = 0;
    int rawSaved = 0;
    std::optional<std::vector<cv::Point2f>> prevSavedCorners;
    auto lastSave = std::chrono::steady_clock::now() - std::chrono::milliseconds(std::max(intervalMs, 1));
    auto lastAutoSkipLog = std::chrono::steady_clock::now() - std::chrono::milliseconds(2000);

    while (saved < count) {
        cv::Mat frame;
        if (useMf) {
            if (!mf.readBgr(frame) || frame.empty()) continue;
        } else {
            if (!cap.read(frame) || frame.empty()) continue;
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

        const double blur = blurScoreVarianceOfLaplacian(gray);

        std::vector<cv::Point2f> corners;
        bool found = false;

        // Prefer SB (more robust / accurate). Fallback to classic for speed/compat.
        found = cv::findChessboardCornersSB(gray, pattern, corners, cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ACCURACY);
        if (!found) {
            found = cv::findChessboardCorners(gray, pattern, corners,
                                              cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        }

        if (found) {
            cv::cornerSubPix(gray,
                             corners,
                             cv::Size(15, 15),
                             cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 80, 1e-12));
            cv::drawChessboardCorners(frame, pattern, corners, true);
        }

        const double shift = found ? meanCornerShift(prevSavedCorners, corners) : 0.0;
        const double bsize = found ? boardNeighborSizePx(corners, pattern) : 0.0;
        const double border = found ? minBorderPx(corners, frame.cols, frame.rows) : 0.0;

        const bool blurOk = (blur >= minBlur);
        const bool shiftOk = (!prevSavedCorners.has_value()) ? true : (shift >= minShiftPx);
        const bool boardOk = (bsize >= minBoard);
        const bool borderOk = (border >= minBorder);
        const bool good = found && blurOk && shiftOk && boardOk && borderOk;

        {
            std::ostringstream ss;
            ss << saved << "/" << count
               << "  auto=" << (autoEnabled ? 1 : 0)
               << "  " << (found ? (good ? "GOOD" : "FOUND") : "NO")
               << "  blur=" << (int)blur;
            if (found) {
                ss << "  shift=" << std::fixed << std::setprecision(1) << (prevSavedCorners.has_value() ? shift : 0.0);
                ss << "  border=" << std::fixed << std::setprecision(1) << border;
                ss << "  size=" << std::fixed << std::setprecision(1) << bsize;
            }

            const cv::Scalar col = good ? cv::Scalar(0, 255, 0) : (found ? cv::Scalar(0, 200, 255) : cv::Scalar(0, 0, 255));
            cv::putText(frame, ss.str(), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 3);
            cv::putText(frame, ss.str(), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, col, 1);

            if (!found) {
                cv::putText(frame, "Searching chessboard...", {10, 70}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }
        }

        cv::imshow(win, frame);

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 27) break;
        if (key == 'a' || key == 'A') {
            autoEnabled = !autoEnabled;
            std::cout << "Auto: " << (autoEnabled ? "ON" : "OFF") << "\n";
        }

        if (key == 's' || key == 'S') {
            const fs::path outPath = outDir / ("raw_" + cv::format("%04d", rawSaved) + ".png");
            const std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, pngCompression};
            if (cv::imwrite(outPath.string(), frameRaw, params)) {
                rawSaved++;
                std::cout << "Saved raw: " << outPath.string() << "\n";
            }
        }

        const auto now = std::chrono::steady_clock::now();
        const bool manualSave = (key == 32);
        const bool autoSave = autoEnabled && (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSave).count() >= intervalMs);

        if ((manualSave || autoSave) && found) {
            const bool shouldLog = manualSave || (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastAutoSkipLog).count() >= 1200);
            if (!blurOk) {
                if (shouldLog) std::cout << "Skip: blur " << blur << " < " << minBlur << (autoSave ? " (auto)" : "") << "\n";
                lastAutoSkipLog = now;
                continue;
            }
            if (!boardOk) {
                if (shouldLog) std::cout << "Skip: board too small (min_board_px=" << minBoard << ")" << (autoSave ? " (auto)" : "") << "\n";
                lastAutoSkipLog = now;
                continue;
            }
            if (!borderOk) {
                if (shouldLog) std::cout << "Skip: too close to border (min_border_px=" << minBorder << ")" << (autoSave ? " (auto)" : "") << "\n";
                lastAutoSkipLog = now;
                continue;
            }
            if (!shiftOk) {
                if (shouldLog) std::cout << "Skip: pose too similar (min_shift_px=" << minShiftPx << ")" << (autoSave ? " (auto)" : "") << "\n";
                lastAutoSkipLog = now;
                continue;
            }

            const fs::path outPath = outDir / ("calib_" + cv::format("%04d", saved) + ".png");
            const std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, pngCompression};
            if (!cv::imwrite(outPath.string(), frameRaw, params)) {
                std::cerr << "ERROR: Failed to write " << outPath.string() << "\n";
                continue;
            }

            prevSavedCorners = corners;
            lastSave = now;
            saved++;
            std::cout << (autoSave ? "Auto-saved: " : "Saved: ") << outPath.string() << "\n";
        } else if (manualSave && !found) {
            std::cout << "Skip: chessboard NOT found\n";
        }
    }

    if (!useMf) cap.release();
    cv::destroyAllWindows();

    std::cout << "Done. Saved " << saved << " images to " << outDir.string() << "\n";
    return 0;
}
