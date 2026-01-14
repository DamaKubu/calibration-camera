#include <opencv2/opencv.hpp>

#include <opencv2/core/cuda.hpp>
#if __has_include(<opencv2/cudaimgproc.hpp>)
#include <opencv2/cudaimgproc.hpp>
#define CAPTURE_MULTIPLE_HAS_CUDAIMGPROC 1
#else
#define CAPTURE_MULTIPLE_HAS_CUDAIMGPROC 0
#endif

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "../mf_camera.h"

namespace fs = std::filesystem;

static std::vector<std::string> splitComma(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == ',') {
            if (!cur.empty()) out.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

static fs::path defaultProjectRoot() {
    fs::path start = fs::current_path();
    fs::path cmake = mfcam::findUpwardsForFile(start, "CMakeLists.txt");
    if (!cmake.empty()) return cmake.parent_path();
    return start;
}

static std::string nowTimestamp() {
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
    ss << std::setw(4) << std::setfill('0') << (tm.tm_year + 1900)
       << std::setw(2) << std::setfill('0') << (tm.tm_mon + 1)
       << std::setw(2) << std::setfill('0') << (tm.tm_mday)
       << "_"
       << std::setw(2) << std::setfill('0') << (tm.tm_hour)
       << std::setw(2) << std::setfill('0') << (tm.tm_min)
       << std::setw(2) << std::setfill('0') << (tm.tm_sec);
    return ss.str();
}

static std::string shotName(int idx) {
    std::ostringstream ss;
    ss << "shot_" << std::setw(4) << std::setfill('0') << idx;
    return ss.str();
}

struct DetectResult {
    bool found = false;
    std::vector<cv::Point2f> corners;
    double meanMotionPx = 1e9;
    double boardSizePx = 0.0;
    double minBorderPx = 0.0;
};

static DetectResult detectChessboard(const cv::Mat& bgr,
                                    const cv::Size& patternSize,
                                    const std::vector<cv::Point2f>* prevCorners,
                                    bool useSB,
                                    int subpixWin,
                                    bool cudaPreprocess,
                                    double detectScale,
                                    bool fastCheck) {
    DetectResult r;

    if (bgr.empty()) return r;

    cv::Mat gray;
    if (cudaPreprocess && CAPTURE_MULTIPLE_HAS_CUDAIMGPROC && cv::cuda::getCudaEnabledDeviceCount() > 0) {
        try {
            cv::cuda::GpuMat d_src, d_gray;
            d_src.upload(bgr);
            if (bgr.channels() == 3) {
                cv::cuda::cvtColor(d_src, d_gray, cv::COLOR_BGR2GRAY);
            } else if (bgr.channels() == 4) {
                cv::cuda::cvtColor(d_src, d_gray, cv::COLOR_BGRA2GRAY);
            } else {
                d_gray = d_src;
            }
            d_gray.download(gray);
        } catch (...) {
            gray.release();
        }
    }

    if (gray.empty()) {
        if (bgr.channels() == 3) {
            cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        } else if (bgr.channels() == 4) {
            cv::cvtColor(bgr, gray, cv::COLOR_BGRA2GRAY);
        } else {
            gray = bgr;
        }
    }

    cv::Mat grayDetect = gray;
    if (detectScale > 0.0 && detectScale < 1.0) {
        cv::resize(gray, grayDetect, cv::Size(), detectScale, detectScale, cv::INTER_AREA);
    }

    std::vector<cv::Point2f> corners;
    bool found = false;

    if (useSB) {
        found = cv::findChessboardCornersSB(grayDetect, patternSize, corners);
        if (!found) {
            int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
            if (fastCheck) flags |= cv::CALIB_CB_FAST_CHECK;
            found = cv::findChessboardCorners(grayDetect, patternSize, corners, flags);
        }
    } else {
        int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
        if (fastCheck) flags |= cv::CALIB_CB_FAST_CHECK;
        found = cv::findChessboardCorners(grayDetect, patternSize, corners, flags);
    }

    if (!found) {
        r.found = false;
        return r;
    }

    if (detectScale > 0.0 && detectScale < 1.0) {
        const float inv = (float)(1.0 / detectScale);
        for (auto& p : corners) {
            p.x *= inv;
            p.y *= inv;
        }
    }

    if (subpixWin > 0) {
        const int w = subpixWin;
        cv::cornerSubPix(gray, corners, cv::Size(w, w), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 1e-3));
    }

    r.found = true;
    r.corners = corners;

    // Stability metric: mean corner motion vs previous.
    if (prevCorners && prevCorners->size() == corners.size()) {
        double sum = 0.0;
        for (size_t i = 0; i < corners.size(); ++i) {
            sum += cv::norm(corners[i] - (*prevCorners)[i]);
        }
        r.meanMotionPx = sum / (double)corners.size();
    } else {
        // No previous corners yet: don't fail the motion gate on the first detection.
        r.meanMotionPx = 0.0;
    }

    // Board size metric: average horizontal/vertical neighbor distance.
    const int cols = patternSize.width;
    const int rows = patternSize.height;
    double sumD = 0.0;
    int cntD = 0;
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            const int idx = y * cols + x;
            if (x + 1 < cols) {
                sumD += cv::norm(corners[idx] - corners[idx + 1]);
                cntD++;
            }
            if (y + 1 < rows) {
                sumD += cv::norm(corners[idx] - corners[idx + cols]);
                cntD++;
            }
        }
    }
    r.boardSizePx = (cntD > 0) ? (sumD / (double)cntD) : 0.0;

    // Border margin metric.
    double minBorder = 1e9;
    for (const auto& p : corners) {
        minBorder = std::min(minBorder, (double)p.x);
        minBorder = std::min(minBorder, (double)p.y);
        minBorder = std::min(minBorder, (double)(bgr.cols - 1) - (double)p.x);
        minBorder = std::min(minBorder, (double)(bgr.rows - 1) - (double)p.y);
    }
    r.minBorderPx = (minBorder == 1e9) ? 0.0 : minBorder;

    return r;
}

int main(int argc, char** argv) {
    // Defaults requested by user
    int patternCols = 8;
    int patternRows = 5;
    double squareMm = 65.0;

    std::vector<std::string> camKeys;
    std::string camerasYml;
    std::string outDirArg;

    int count = 60;
    int intervalMs = 0;
    bool autoMode = true;
    int minAutoSaveMs = 350;

    int detectEvery = 2;          // run detection every N frames
    double detectScale = 0.5;     // downscale grayscale for detection
    bool fastCheck = false;       // may reduce CPU, sometimes less reliable

    int cellW = 480;
    int cellH = 270;
    int sleepMs = 1;

    // Quality gates
    double maxMotionPx = 0.25;      // stability threshold (mean corner motion)
    int stableFrames = 6;           // require N consecutive stable frames
    double minBoardSizePx = 20.0;   // board must be reasonably large
    double minBorderPx = 20.0;      // corners not too close to border

    int pngCompression = 0;
    bool useSB = true;
    int subpixWin = 5;

    bool cudaPreprocess = false;

    unsigned reqWidth = 0;
    unsigned reqHeight = 0;
    unsigned reqFps = 0;

    auto need = [&](int& i, const char* name) -> std::string {
        if (i + 1 >= argc) {
            std::cerr << "Missing value for " << name << "\n";
            std::exit(2);
        }
        return argv[++i];
    };

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];

        if (a == "--cams") {
            camKeys = splitComma(need(i, "--cams"));
        } else if (a == "--cam") {
            camKeys.push_back(need(i, "--cam"));
        } else if (a == "--cameras-yml") {
            camerasYml = need(i, "--cameras-yml");
        } else if (a == "--out-dir") {
            outDirArg = need(i, "--out-dir");
        } else if (a == "--count") {
            count = std::stoi(need(i, "--count"));
        } else if (a == "--interval-ms") {
            intervalMs = std::stoi(need(i, "--interval-ms"));
        } else if (a == "--auto") {
            autoMode = true;
        } else if (a == "--manual") {
            autoMode = false;
        } else if (a == "--min-auto-save-ms") {
            minAutoSaveMs = std::stoi(need(i, "--min-auto-save-ms"));
        } else if (a == "--detect-every") {
            detectEvery = std::stoi(need(i, "--detect-every"));
        } else if (a == "--detect-scale") {
            detectScale = std::stod(need(i, "--detect-scale"));
        } else if (a == "--fast") {
            fastCheck = true;
        } else if (a == "--pattern-cols") {
            patternCols = std::stoi(need(i, "--pattern-cols"));
        } else if (a == "--pattern-rows") {
            patternRows = std::stoi(need(i, "--pattern-rows"));
        } else if (a == "--square-mm") {
            squareMm = std::stod(need(i, "--square-mm"));
        } else if (a == "--max-motion-px") {
            maxMotionPx = std::stod(need(i, "--max-motion-px"));
        } else if (a == "--stable-frames") {
            stableFrames = std::stoi(need(i, "--stable-frames"));
        } else if (a == "--min-board-px") {
            minBoardSizePx = std::stod(need(i, "--min-board-px"));
        } else if (a == "--min-border-px") {
            minBorderPx = std::stod(need(i, "--min-border-px"));
        } else if (a == "--png-compression") {
            pngCompression = std::stoi(need(i, "--png-compression"));
            if (pngCompression < 0) pngCompression = 0;
            if (pngCompression > 9) pngCompression = 9;
        } else if (a == "--no-sb") {
            useSB = false;
        } else if (a == "--subpix-win") {
            subpixWin = std::stoi(need(i, "--subpix-win"));
        } else if (a == "--cuda-preprocess") {
            cudaPreprocess = true;
        } else if (a == "--cell-w") {
            cellW = std::stoi(need(i, "--cell-w"));
        } else if (a == "--cell-h") {
            cellH = std::stoi(need(i, "--cell-h"));
        } else if (a == "--sleep-ms") {
            sleepMs = std::stoi(need(i, "--sleep-ms"));
        } else if (a == "--width") {
            reqWidth = (unsigned)std::stoul(need(i, "--width"));
        } else if (a == "--height") {
            reqHeight = (unsigned)std::stoul(need(i, "--height"));
        } else if (a == "--fps") {
            reqFps = (unsigned)std::stoul(need(i, "--fps"));
        } else if (a == "--help" || a == "-h") {
            std::cout
                << "capture_multiple\n"
                << "Usage:\n"
                << "  capture_multiple.exe --cams cam1,cam2,cam3 --count 60 [--manual] [--interval-ms 0]\n"
                << "  capture_multiple.exe --cams cam1,cam2 --count 60 --fast --detect-scale 0.5 --detect-every 2\n"
                << "\n"
                << "Defaults:\n"
                << "  pattern_cols=8 pattern_rows=5 square_mm=65\n"
                << "  auto=ON (saves only when ALL cameras are GOOD + stable)\n"
                << "\n"
                << "Controls:\n"
                << "  [Space]=save shot   [A]=toggle auto   [Esc]=quit\n";
            return 0;
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            return 2;
        }
    }

    if ((int)camKeys.size() < 2 || (int)camKeys.size() > 6) {
        std::cerr << "ERROR: Provide 2..6 cameras using --cams or repeated --cam\n";
        return 2;
    }

    if (count <= 0) {
        std::cerr << "ERROR: --count must be > 0\n";
        return 2;
    }

    if (detectEvery < 1) detectEvery = 1;
    if (!(detectScale > 0.0 && detectScale <= 1.0)) detectScale = 1.0;
    if (minAutoSaveMs < 0) minAutoSaveMs = 0;
    if (cellW < 160) cellW = 160;
    if (cellH < 120) cellH = 120;
    if (sleepMs < 0) sleepMs = 0;

    const cv::Size patternSize(patternCols, patternRows);

    fs::path projectRoot;
    fs::path ymlPath;
    if (!camerasYml.empty()) {
        ymlPath = fs::absolute(fs::path(camerasYml));
        projectRoot = ymlPath.parent_path();
    } else {
        projectRoot = defaultProjectRoot();
        ymlPath = mfcam::findUpwardsForFile(fs::current_path(), "cameras.yml");
    }

    if (ymlPath.empty()) {
        std::cerr << "ERROR: cameras.yml not found. Provide --cameras-yml <path>\n";
        return 2;
    }

    std::vector<std::string> symbolicLinks;
    symbolicLinks.resize(camKeys.size());
    for (size_t i = 0; i < camKeys.size(); ++i) {
        if (!mfcam::loadSymbolicLinkFromCamerasYml(ymlPath, camKeys[i], symbolicLinks[i])) {
            std::cerr << "ERROR: Could not find '" << camKeys[i] << "'->symbolic_link in " << ymlPath.string() << "\n";
            return 2;
        }
    }

    // Output folder (timestamped by default)
    fs::path outDir;
    if (!outDirArg.empty()) {
        fs::path p(outDirArg);
        outDir = p.is_absolute() ? p : (projectRoot / p);
    } else {
        outDir = projectRoot / "data" / "extrinsic_multi" / ("session_" + nowTimestamp());
    }
    fs::create_directories(outDir);

    // Write a session manifest
    {
        cv::FileStorage fs((outDir / "session.yml").string(), cv::FileStorage::WRITE);
        const std::string buildStamp = std::string(__DATE__) + " " + std::string(__TIME__);
        fs << "build" << buildStamp;
        fs << "pattern_cols" << patternCols;
        fs << "pattern_rows" << patternRows;
        fs << "square_mm" << squareMm;
        fs << "cam_count" << (int)camKeys.size();
        fs << "notes" << "Each shot is saved as shot_XXXX_<cam>.png with a matching shot_XXXX.yml.";
        fs << "cameras" << "[";
        for (size_t i = 0; i < camKeys.size(); ++i) {
            fs << "{";
            fs << "key" << camKeys[i];
            fs << "symbolic_link" << symbolicLinks[i];
            fs << "}";
        }
        fs << "]";
    }

    // Open cameras
    std::vector<mfcam::Camera> cams(camKeys.size());
    mfcam::OpenOptions opts;
    opts.width = reqWidth;
    opts.height = reqHeight;
    opts.fps = reqFps;

    for (size_t i = 0; i < cams.size(); ++i) {
        if (!cams[i].openSymbolicLink(symbolicLinks[i], opts)) {
            std::cerr << "ERROR: Cannot open " << camKeys[i] << " by symbolic_link\n";
            std::cerr << "  symbolic_link: " << symbolicLinks[i] << "\n";
            return 1;
        }
    }

    std::cout << "Saving to: " << outDir.string() << "\n";
    std::cout << "Cameras.yml: " << ymlPath.string() << "\n";
    std::cout << "Cameras: ";
    for (size_t i = 0; i < camKeys.size(); ++i) {
        std::cout << camKeys[i] << (i + 1 < camKeys.size() ? ", " : "\n");
    }

    std::cout << "Instructions for high precision:\n";
    std::cout << "  - Keep the chessboard visible in ALL cameras at once.\n";
    std::cout << "  - Move it across the shared overlap region (center and edges).\n";
    std::cout << "  - Hold the board steady until status becomes GOOD, then save.\n";
    std::cout << "  - For <1px precision: use many poses, avoid motion blur, keep corners away from borders.\n";
    if (cudaPreprocess) {
        std::cout << "CUDA preprocess: ON (" << (CAPTURE_MULTIPLE_HAS_CUDAIMGPROC ? "cudaimgproc available" : "cudaimgproc missing") << ")\n";
    }

    const std::string win = "capture_multiple";
    cv::namedWindow(win, cv::WINDOW_NORMAL);

    std::vector<cv::Mat> frames(camKeys.size());
    std::vector<std::vector<cv::Point2f>> prevCorners(camKeys.size());

    std::vector<DetectResult> detCache(cams.size());

    int frameCounter = 0;

    int goodStreak = 0;
    int saved = 0;
    int shotIdx = 0;

    auto lastSave = std::chrono::steady_clock::now();

    while (true) {
        frameCounter++;

        // Grab frames
        for (size_t i = 0; i < cams.size(); ++i) {
            cv::Mat f;
            if (!cams[i].readBgr(f)) {
                std::cerr << "ERROR: read failed for " << camKeys[i] << "\n";
                return 1;
            }
            frames[i] = f;
        }

        // Detect per cam
        std::vector<DetectResult> det(cams.size());
        bool allFound = true;
        bool allQuality = true;

        const bool doDetect = (frameCounter % detectEvery) == 0;

        for (size_t i = 0; i < cams.size(); ++i) {
            if (doDetect) {
                detCache[i] = detectChessboard(frames[i], patternSize, prevCorners[i].empty() ? nullptr : &prevCorners[i],
                                               useSB, subpixWin, cudaPreprocess, detectScale, fastCheck);
            }
            det[i] = detCache[i];
            allFound = allFound && det[i].found;

            bool q = det[i].found;
            q = q && det[i].boardSizePx >= minBoardSizePx;
            q = q && det[i].minBorderPx >= minBorderPx;
            q = q && det[i].meanMotionPx <= maxMotionPx;
            allQuality = allQuality && q;
        }

        if (allFound && allQuality) {
            goodStreak++;
        } else {
            goodStreak = 0;
        }

        const bool goodFrame = (goodStreak >= stableFrames);

        // Build mosaic
        const int n = (int)frames.size();
        int cols = (n <= 3) ? n : 3;
        int rows = (n + cols - 1) / cols;

        cv::Mat mosaic(rows * cellH, cols * cellW, CV_8UC3, cv::Scalar(30, 30, 30));

        for (int i = 0; i < n; ++i) {
            int r = i / cols;
            int c = i % cols;
            cv::Rect roi(c * cellW, r * cellH, cellW, cellH);

            cv::Mat view;
            cv::resize(frames[i], view, roi.size());

            // Draw chessboard overlay
            if (det[(size_t)i].found) {
                // Need scaled corners for resized view
                std::vector<cv::Point2f> scaled = det[(size_t)i].corners;
                const double sx = (double)roi.width / (double)frames[(size_t)i].cols;
                const double sy = (double)roi.height / (double)frames[(size_t)i].rows;
                for (auto& p : scaled) {
                    p.x = (float)(p.x * sx);
                    p.y = (float)(p.y * sy);
                }
                cv::drawChessboardCorners(view, patternSize, scaled, true);
            }

            const bool q = det[(size_t)i].found && det[(size_t)i].boardSizePx >= minBoardSizePx && det[(size_t)i].minBorderPx >= minBorderPx && det[(size_t)i].meanMotionPx <= maxMotionPx;

            cv::Scalar colStatus = q ? cv::Scalar(0, 200, 0) : (det[(size_t)i].found ? cv::Scalar(0, 180, 255) : cv::Scalar(0, 0, 255));
            std::string status = det[(size_t)i].found ? (q ? "GOOD" : "FOUND") : "NO";

            std::ostringstream ss;
            ss << camKeys[(size_t)i] << "  " << status;
            if (det[(size_t)i].found) {
                ss << "  motion=" << std::fixed << std::setprecision(3) << det[(size_t)i].meanMotionPx;
                ss << "  size=" << std::fixed << std::setprecision(1) << det[(size_t)i].boardSizePx;
            }

            cv::putText(view, ss.str(), cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 3);
            cv::putText(view, ss.str(), cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6, colStatus, 1);

            view.copyTo(mosaic(roi));
        }

        // Global status + instructions
        const int elapsedSinceSaveUiMs = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                                              std::chrono::steady_clock::now() - lastSave)
                                              .count();
        {
            std::ostringstream ss;
            ss << "ALL: " << (allFound ? "FOUND" : "NOT FOUND")
               << "   quality: " << (allQuality ? "OK" : "WAIT")
               << "   streak: " << goodStreak << "/" << stableFrames
               << "   saved: " << saved << "/" << count
               << "   auto: " << (autoMode ? "ON" : "OFF")
               << "   dt: " << elapsedSinceSaveUiMs << "ms";

            cv::putText(mosaic, ss.str(), cv::Point(10, mosaic.rows - 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 3);
            cv::putText(mosaic, ss.str(), cv::Point(10, mosaic.rows - 40), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        goodFrame ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255), 1);

            const std::string help = "[Space]=save  [A]=auto  [Esc]=quit";
            cv::putText(mosaic, help, cv::Point(10, mosaic.rows - 12), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 3);
            cv::putText(mosaic, help, cv::Point(10, mosaic.rows - 12), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(220, 220, 220), 1);
        }

        cv::imshow(win, mosaic);

        if (sleepMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
        }

        const auto now = std::chrono::steady_clock::now();
        const int elapsedSinceSaveMs = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSave).count();

        bool doSave = false;
        if (intervalMs > 0 && elapsedSinceSaveMs >= intervalMs) {
            doSave = true;
        }
        if (autoMode && goodFrame && elapsedSinceSaveMs >= minAutoSaveMs) {
            doSave = true;
        }

        int key = cv::waitKey(1) & 0xFF;
        if (key == 27) break;
        if (key == 'a' || key == 'A') autoMode = !autoMode;
        if (key == 32) doSave = true;

        if (doSave) {
            if (!allFound) {
                // Don't save junk
                continue;
            }

            const std::string base = shotName(shotIdx);

            std::vector<int> params;
            params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            params.push_back(pngCompression);

            bool ok = true;
            for (size_t i = 0; i < frames.size(); ++i) {
                fs::path outPath = outDir / (base + "_" + camKeys[i] + ".png");
                if (!cv::imwrite(outPath.string(), frames[i], params)) {
                    ok = false;
                    std::cerr << "ERROR: Failed to write " << outPath.string() << "\n";
                    break;
                }
            }

            if (ok) {
                // Write per-shot manifest (keeps YAML valid without needing APPEND)
                cv::FileStorage fsShot((outDir / (base + ".yml")).string(), cv::FileStorage::WRITE);
                const std::string buildStamp = std::string(__DATE__) + " " + std::string(__TIME__);
                fsShot << "build" << buildStamp;
                fsShot << "shot" << shotIdx;
                fsShot << "pattern_cols" << patternCols;
                fsShot << "pattern_rows" << patternRows;
                fsShot << "square_mm" << squareMm;
                fsShot << "files" << "[";
                for (size_t i = 0; i < camKeys.size(); ++i) {
                    fsShot << (base + "_" + camKeys[i] + ".png");
                }
                fsShot << "]";
                fsShot << "cameras" << "[";
                for (size_t i = 0; i < camKeys.size(); ++i) {
                    fsShot << "{";
                    fsShot << "key" << camKeys[i];
                    fsShot << "symbolic_link" << symbolicLinks[i];
                    fsShot << "}";
                }
                fsShot << "]";

                std::cout << "Saved " << base << " (" << (saved + 1) << "/" << count << ")\n";
                saved++;
                shotIdx++;
                goodStreak = 0;
                lastSave = std::chrono::steady_clock::now();

                if (saved >= count) break;
            }
        }

        // update prev corners
        for (size_t i = 0; i < det.size(); ++i) {
            if (det[i].found) prevCorners[i] = det[i].corners;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
