#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static fs::path findUpwardsForFile(fs::path start, const std::string& filename) {
    start = fs::absolute(start);
    fs::path cur = start;
    for (int depth = 0; depth < 12; ++depth) {
        const fs::path cand = cur / filename;
        if (fs::exists(cand)) return cand;
        if (!cur.has_parent_path()) break;
        const fs::path parent = cur.parent_path();
        if (parent == cur) break;
        cur = parent;
    }
    return fs::path();
}

static fs::path defaultProjectRoot() {
    const fs::path cmake = findUpwardsForFile(fs::current_path(), "CMakeLists.txt");
    if (!cmake.empty()) return cmake.parent_path();
    return fs::current_path();
}

static bool readIntrinsics(const fs::path& intrPath, cv::Mat& K, cv::Mat& D) {
    K.release();
    D.release();
    cv::FileStorage fs(intrPath.string(), cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;
    fs.release();
    if (K.empty() || K.rows != 3 || K.cols != 3) return false;
    if (D.empty()) return false;
    return true;
}

static std::vector<cv::Point3f> makeObjectPoints(const cv::Size& pattern, double squareMm) {
    std::vector<cv::Point3f> obj;
    obj.reserve((size_t)pattern.width * (size_t)pattern.height);
    for (int y = 0; y < pattern.height; ++y) {
        for (int x = 0; x < pattern.width; ++x) {
            obj.push_back(cv::Point3f((float)(x * squareMm), (float)(y * squareMm), 0.0f));
        }
    }
    return obj;
}

static bool detectCorners(const cv::Mat& bgr, const cv::Size& pattern, std::vector<cv::Point2f>& corners) {
    corners.clear();
    if (bgr.empty()) return false;

    cv::Mat gray;
    if (bgr.channels() == 3) {
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    } else if (bgr.channels() == 4) {
        cv::cvtColor(bgr, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = bgr;
    }

    bool found = cv::findChessboardCornersSB(
        gray,
        pattern,
        corners,
        cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
    if (!found) {
        found = cv::findChessboardCorners(
            gray,
            pattern,
            corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    }
    if (!found) return false;

    cv::cornerSubPix(
        gray,
        corners,
        cv::Size(15, 15),
        cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 200, 1e-12));

    return true;
}

static bool parseShotIndex(const std::string& filename, int& idxOut) {
    // expected: shot_XXXX_...
    if (filename.rfind("shot_", 0) != 0) return false;
    if (filename.size() < 9) return false;
    const std::string num = filename.substr(5, 4);
    for (char c : num) {
        if (c < '0' || c > '9') return false;
    }
    idxOut = std::stoi(num);
    return true;
}

int main(int argc, char** argv) {
    std::string cameraId = "cam0";
    cv::Size pattern(8, 5);
    double squareMm = 65.0;

    std::string sessionDir;
    std::string refCam;
    fs::path projectRoot = defaultProjectRoot();
    fs::path intrinsicsRoot = projectRoot / "data";
    std::string intrinsicsPrefix = "calib_";

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                std::exit(2);
            }
            return argv[++i];
        };

        if (a == "--session") {
            sessionDir = need("--session");
        } else if (a == "--ref") {
            refCam = need("--ref");
        } else if (a == "--intrinsics-root") {
            intrinsicsRoot = fs::path(need("--intrinsics-root"));
            if (!intrinsicsRoot.is_absolute()) intrinsicsRoot = projectRoot / intrinsicsRoot;
        } else if (a == "--intrinsics-prefix") {
            intrinsicsPrefix = need("--intrinsics-prefix");
        } else if (a == "--camera-id") {
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
                << "  (single camera pose)\\n"
                << "    extrinsic.exe --camera-id cam0 [--pattern 8x5] [--square-mm 65]\\n"
                << "\\n"
                << "  (multi-camera session extrinsics)\\n"
                << "    extrinsic.exe --session data/extrinsic_multi/session_0 --ref cam1\\n"
                << "    extrinsic.exe --session data/extrinsic_multi/session_0 --ref cam1 --intrinsics-root data\\n"
                << "\\n"
                << "Single camera mode:\\n"
                << "  Reads images from: data/<camera_id>/img_*.png\\n"
                << "  Reads intrinsics from: data/<camera_id>/intrinsic.yml\\n"
                << "  Writes extrinsics to: data/<camera_id>/extrinsic.yml\\n"
                << "\\n"
                << "Multi-camera mode:\\n"
                << "  Reads: <session>/session.yml and shot_XXXX_<cam>.png files.\\n"
                << "  Reads intrinsics from: <intrinsics_root>/<intrinsics_prefix><cam>/intrinsic.yml\\n"
                << "  Writes: <session>/extrinsics.yml (cam_ref -> cam_i)\\n";
            return 0;
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            return 2;
        }
    }

    // ---- Multi-camera session mode ----
    if (!sessionDir.empty()) {
        fs::path sessionPath(sessionDir);
        if (!sessionPath.is_absolute()) sessionPath = projectRoot / sessionPath;
        sessionPath = fs::absolute(sessionPath);

        const fs::path sessionYml = sessionPath / "session.yml";
        if (!fs::exists(sessionYml)) {
            std::cerr << "ERROR: Missing session.yml: " << sessionYml.string() << "\n";
            return 1;
        }

        cv::FileStorage fsSession(sessionYml.string(), cv::FileStorage::READ);
        if (!fsSession.isOpened()) {
            std::cerr << "ERROR: Cannot open session.yml: " << sessionYml.string() << "\n";
            return 1;
        }

        int patternCols = 0, patternRows = 0;
        double squareMmFromSession = 0.0;
        fsSession["pattern_cols"] >> patternCols;
        fsSession["pattern_rows"] >> patternRows;
        fsSession["square_mm"] >> squareMmFromSession;
        if (patternCols > 0 && patternRows > 0) pattern = cv::Size(patternCols, patternRows);
        if (squareMmFromSession > 0.0) squareMm = squareMmFromSession;

        std::vector<std::string> cams;
        {
            cv::FileNode n = fsSession["cameras"];
            if (n.type() != cv::FileNode::SEQ) {
                std::cerr << "ERROR: session.yml missing 'cameras' list\n";
                return 1;
            }
            for (auto it = n.begin(); it != n.end(); ++it) {
                std::string key;
                (*it)["key"] >> key;
                if (!key.empty()) cams.push_back(key);
            }
        }
        fsSession.release();

        if (cams.size() < 2) {
            std::cerr << "ERROR: session has <2 cameras\n";
            return 1;
        }
        if (refCam.empty()) refCam = cams.front();

        int refIdx = -1;
        for (int i = 0; i < (int)cams.size(); ++i) {
            if (cams[(size_t)i] == refCam) {
                refIdx = i;
                break;
            }
        }
        if (refIdx < 0) {
            std::cerr << "ERROR: --ref " << refCam << " not found in session cameras\n";
            return 1;
        }

        struct CamCalib {
            cv::Mat K;
            cv::Mat D;
        };
        std::vector<CamCalib> calib(cams.size());
        for (size_t i = 0; i < cams.size(); ++i) {
            const fs::path intrPath = intrinsicsRoot / (intrinsicsPrefix + cams[i]) / "intrinsic.yml";
            if (!readIntrinsics(intrPath, calib[i].K, calib[i].D)) {
                std::cerr << "ERROR: Missing/invalid intrinsics for " << cams[i] << ": " << intrPath.string() << "\n";
                return 1;
            }
        }

        // Collect shot indices by scanning PNG names.
        std::set<int> shotIdxs;
        for (const auto& e : fs::directory_iterator(sessionPath)) {
            if (!e.is_regular_file()) continue;
            const fs::path p = e.path();
            if (p.extension().string() != ".png") continue;
            int idx = -1;
            if (!parseShotIndex(p.filename().string(), idx)) continue;
            shotIdxs.insert(idx);
        }
        if (shotIdxs.empty()) {
            std::cerr << "ERROR: No shot_XXXX_*.png found in " << sessionPath.string() << "\n";
            return 1;
        }

        const std::vector<cv::Point3f> obj = makeObjectPoints(pattern, squareMm);

        struct Shot {
            int idx = -1;
            cv::Size imageSize;
            std::vector<bool> found;
            std::vector<std::vector<cv::Point2f>> corners;
        };

        std::vector<Shot> shots;
        shots.reserve(shotIdxs.size());

        for (int idx : shotIdxs) {
            Shot s;
            s.idx = idx;
            s.found.assign(cams.size(), false);
            s.corners.resize(cams.size());

            bool anyRead = false;
            for (size_t ci = 0; ci < cams.size(); ++ci) {
                std::ostringstream name;
                name << "shot_" << std::setw(4) << std::setfill('0') << idx << "_" << cams[ci] << ".png";
                const fs::path imgPath = sessionPath / name.str();
                cv::Mat img = cv::imread(imgPath.string(), cv::IMREAD_COLOR);
                if (img.empty()) continue;
                anyRead = true;
                if (s.imageSize.width == 0) s.imageSize = img.size();

                std::vector<cv::Point2f> pts;
                if (detectCorners(img, pattern, pts)) {
                    s.found[ci] = true;
                    s.corners[ci] = std::move(pts);
                }
            }
            if (!anyRead) continue;
            shots.push_back(std::move(s));
        }

        const int minViews = 20;
        const cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 1000, 1e-12);
        const int flags = cv::CALIB_FIX_INTRINSIC;

        const fs::path outPath = sessionPath / "extrinsics.yml";
        cv::FileStorage out(outPath.string(), cv::FileStorage::WRITE);
        out << "session" << sessionPath.string();
        out << "ref_cam" << refCam;
        out << "pattern_cols" << pattern.width;
        out << "pattern_rows" << pattern.height;
        out << "square_mm" << squareMm;
        out << "intrinsics_root" << intrinsicsRoot.string();
        out << "intrinsics_prefix" << intrinsicsPrefix;

        out << "cameras" << "[";
        for (const auto& c : cams) out << c;
        out << "]";

        out << "pairs" << "[";

        for (size_t ci = 0; ci < cams.size(); ++ci) {
            if ((int)ci == refIdx) continue;

            std::vector<std::vector<cv::Point3f>> objectPoints;
            std::vector<std::vector<cv::Point2f>> imgRef;
            std::vector<std::vector<cv::Point2f>> imgOther;
            cv::Size imageSize;

            for (const auto& s : shots) {
                if (!s.found[(size_t)refIdx] || !s.found[ci]) continue;
                objectPoints.push_back(obj);
                imgRef.push_back(s.corners[(size_t)refIdx]);
                imgOther.push_back(s.corners[ci]);
                if (imageSize.width == 0) imageSize = s.imageSize;
            }

            if ((int)objectPoints.size() < minViews) {
                std::cerr << "ERROR: Not enough views for pair " << refCam << " -> " << cams[ci]
                          << " (have " << objectPoints.size() << ", need >= " << minViews << ")\n";
                return 1;
            }

            cv::Mat R, T, E, F;
            double rms = cv::stereoCalibrate(
                objectPoints,
                imgRef,
                imgOther,
                calib[(size_t)refIdx].K,
                calib[(size_t)refIdx].D,
                calib[ci].K,
                calib[ci].D,
                imageSize,
                R,
                T,
                E,
                F,
                flags,
                criteria);

            out << "{";
            out << "cam1" << refCam;
            out << "cam2" << cams[ci];
            out << "views" << (int)objectPoints.size();
            out << "rms" << rms;
            out << "R" << R;
            out << "T" << T;
            out << "baseline_mm" << cv::norm(T);
            out << "E" << E;
            out << "F" << F;
            out << "}";

            std::cout << "Pair " << refCam << " -> " << cams[ci]
                      << ": views=" << objectPoints.size() << " RMS=" << rms
                      << " baseline_mm=" << cv::norm(T) << "\n";
        }

        out << "]";
        out.release();

        std::cout << "Wrote: " << outPath.string() << "\n";
        return 0;
    }

    const fs::path dir = fs::path("data") / cameraId;
    const fs::path intrPath = dir / "intrinsic.yml";
    const fs::path outPath = dir / "extrinsic.yml";

    cv::Mat K, D;
    if (!readIntrinsics(intrPath, K, D)) {
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

    std::vector<cv::Point3f> obj = makeObjectPoints(pattern, squareMm);

    cv::Mat rvec, tvec;
    std::string usedImage;

    for (const auto& p : files) {
        cv::Mat img = cv::imread(p.string(), cv::IMREAD_COLOR);
        if (img.empty()) continue;

        std::vector<cv::Point2f> corners;
        if (!detectCorners(img, pattern, corners)) continue;

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
