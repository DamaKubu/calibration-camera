#include <opencv2/opencv.hpp>

#include <filesystem>
#include <iostream>
#include <map>
#include <regex>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static void usage() {
    std::cout
        << "stereo_extrinsic_from_pairs\n"
        << "Usage:\n"
        << "  stereo_extrinsic_from_pairs.exe --pairs-dir <dir> --cam1 <key> --cam2 <key> \\\n             --intr1 <intr1.yml> --intr2 <intr2.yml> --pattern 8x5 --square-mm 65 --output <out.yml> [--show]\n";
}

static cv::Size parsePattern(const std::string& s) {
    const size_t x = s.find('x');
    if (x == std::string::npos) return {};
    return cv::Size(std::stoi(s.substr(0, x)), std::stoi(s.substr(x + 1)));
}

static bool loadIntrinsics(const std::string& path, cv::Mat& K, cv::Mat& D, cv::Size& imageSize) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;

    int w = 0, h = 0;
    if (!fs["image_width"].empty()) fs["image_width"] >> w;
    if (!fs["image_height"].empty()) fs["image_height"] >> h;
    imageSize = (w > 0 && h > 0) ? cv::Size(w, h) : cv::Size();

    return !K.empty() && !D.empty();
}

static int flagsForDistortion(const cv::Mat& D) {
    const int n = (int)D.total();
    int flags = 0;
    if (n >= 8) flags |= cv::CALIB_RATIONAL_MODEL;
    if (n >= 12) flags |= cv::CALIB_THIN_PRISM_MODEL;
    if (n >= 14) flags |= cv::CALIB_TILTED_MODEL;
    return flags;
}

static std::vector<cv::Point3f> makeObjectPoints(const cv::Size& patternSize, double squareMm) {
    std::vector<cv::Point3f> obj;
    obj.reserve((size_t)patternSize.area());
    for (int y = 0; y < patternSize.height; ++y) {
        for (int x = 0; x < patternSize.width; ++x) {
            obj.emplace_back((float)(x * squareMm), (float)(y * squareMm), 0.0f);
        }
    }
    return obj;
}

static bool findCorners(const cv::Mat& bgr, const cv::Size& patternSize, std::vector<cv::Point2f>& corners) {
    corners.clear();
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    bool ok = cv::findChessboardCornersSB(gray, patternSize, corners);
    if (!ok) {
        ok = cv::findChessboardCorners(gray, patternSize, corners,
                                       cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    }
    if (!ok) return false;

    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 1e-3));
    return true;
}

// Accept files like: shot_0000_cam1.png / shot_0000_cam2.png
static bool collectShotPairs(const fs::path& dir, const std::string& cam1, const std::string& cam2,
                             std::vector<std::pair<fs::path, fs::path>>& outPairs) {
    outPairs.clear();
    if (!fs::exists(dir) || !fs::is_directory(dir)) return false;

    std::map<std::string, fs::path> a;
    std::map<std::string, fs::path> b;

    auto isPng = [](const fs::path& p) {
        auto e = p.extension().string();
        for (auto& c : e) c = (char)tolower((unsigned char)c);
        return e == ".png";
    };

    const std::regex re(R"(^(shot_\d{4})_(.+)\.png$)", std::regex::icase);

    for (const auto& it : fs::directory_iterator(dir)) {
        if (!it.is_regular_file()) continue;
        const fs::path p = it.path();
        if (!isPng(p)) continue;

        const std::string name = p.filename().string();
        std::smatch m;
        if (!std::regex_match(name, m, re)) continue;

        const std::string shot = m[1].str();
        const std::string key = m[2].str();

        if (_stricmp(key.c_str(), cam1.c_str()) == 0) a[shot] = p;
        if (_stricmp(key.c_str(), cam2.c_str()) == 0) b[shot] = p;
    }

    for (const auto& kv : a) {
        auto it = b.find(kv.first);
        if (it != b.end()) {
            outPairs.emplace_back(kv.second, it->second);
        }
    }

    return !outPairs.empty();
}

int main(int argc, char** argv) {
    std::string pairsDir;
    std::string cam1Key;
    std::string cam2Key;
    std::string intr1;
    std::string intr2;
    std::string patternStr = "8x5";
    double squareMm = 65.0;
    std::string output;
    bool show = false;

    auto need = [&](int& i, const char* name) -> std::string {
        if (i + 1 >= argc) {
            std::cerr << "Missing value for " << name << "\n";
            std::exit(2);
        }
        return argv[++i];
    };

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--pairs-dir") pairsDir = need(i, "--pairs-dir");
        else if (a == "--cam1") cam1Key = need(i, "--cam1");
        else if (a == "--cam2") cam2Key = need(i, "--cam2");
        else if (a == "--intr1") intr1 = need(i, "--intr1");
        else if (a == "--intr2") intr2 = need(i, "--intr2");
        else if (a == "--pattern") patternStr = need(i, "--pattern");
        else if (a == "--square-mm") squareMm = std::stod(need(i, "--square-mm"));
        else if (a == "--output") output = need(i, "--output");
        else if (a == "--show") show = true;
        else if (a == "--help" || a == "-h") {
            usage();
            return 0;
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            usage();
            return 2;
        }
    }

    if (pairsDir.empty() || cam1Key.empty() || cam2Key.empty() || intr1.empty() || intr2.empty() || output.empty()) {
        std::cerr << "ERROR: missing required args\n";
        usage();
        return 2;
    }

    const cv::Size patternSize = parsePattern(patternStr);
    if (patternSize.width <= 0 || patternSize.height <= 0) {
        std::cerr << "ERROR: bad --pattern, expected like 8x5\n";
        return 2;
    }

    cv::Mat K1, D1, K2, D2;
    cv::Size size1, size2;
    if (!loadIntrinsics(intr1, K1, D1, size1)) {
        std::cerr << "ERROR: failed to load intr1: " << intr1 << "\n";
        return 1;
    }
    if (!loadIntrinsics(intr2, K2, D2, size2)) {
        std::cerr << "ERROR: failed to load intr2: " << intr2 << "\n";
        return 1;
    }

    cv::Size imageSize = !size1.empty() ? size1 : size2;
    if (imageSize.empty()) {
        // fall back to actual image size later
        imageSize = cv::Size();
    }

    std::vector<std::pair<fs::path, fs::path>> pairs;
    if (!collectShotPairs(fs::path(pairsDir), cam1Key, cam2Key, pairs)) {
        std::cerr << "ERROR: no pairs found in " << pairsDir << " for keys " << cam1Key << " and " << cam2Key << "\n";
        std::cerr << "Expected files like shot_0000_" << cam1Key << ".png and shot_0000_" << cam2Key << ".png\n";
        return 2;
    }

    const std::vector<cv::Point3f> obj = makeObjectPoints(patternSize, squareMm);

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> img1;
    std::vector<std::vector<cv::Point2f>> img2;

    int used = 0;
    for (const auto& pr : pairs) {
        cv::Mat a = cv::imread(pr.first.string(), cv::IMREAD_COLOR);
        cv::Mat b = cv::imread(pr.second.string(), cv::IMREAD_COLOR);
        if (a.empty() || b.empty()) continue;

        if (imageSize.empty()) imageSize = a.size();

        std::vector<cv::Point2f> c1, c2;
        bool ok1 = findCorners(a, patternSize, c1);
        bool ok2 = findCorners(b, patternSize, c2);
        if (!ok1 || !ok2) {
            continue;
        }

        objectPoints.push_back(obj);
        img1.push_back(c1);
        img2.push_back(c2);
        used++;

        if (show) {
            cv::Mat va = a.clone();
            cv::Mat vb = b.clone();
            cv::drawChessboardCorners(va, patternSize, c1, true);
            cv::drawChessboardCorners(vb, patternSize, c2, true);
            cv::imshow("cam1", va);
            cv::imshow("cam2", vb);
            int k = cv::waitKey(1);
            if ((k & 0xFF) == 27) show = false;
        }
    }

    if (show) {
        cv::destroyWindow("cam1");
        cv::destroyWindow("cam2");
    }

    if (used < 5) {
        std::cerr << "ERROR: not enough valid pairs with chessboard detected. used=" << used << "\n";
        return 2;
    }

    cv::Mat R, T, E, F;
    int flags = cv::CALIB_FIX_INTRINSIC;
    flags |= flagsForDistortion(D1);
    flags |= flagsForDistortion(D2);

    const cv::TermCriteria term(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-7);
    const double rms = cv::stereoCalibrate(objectPoints, img1, img2,
                                          K1, D1, K2, D2,
                                          imageSize, R, T, E, F,
                                          flags, term);

    cv::FileStorage fsOut(output, cv::FileStorage::WRITE);
    fsOut << "build" << __DATE__ << " " << __TIME__;
    fsOut << "pairs_dir" << pairsDir;
    fsOut << "cam1" << cam1Key;
    fsOut << "cam2" << cam2Key;
    fsOut << "pattern_cols" << patternSize.width;
    fsOut << "pattern_rows" << patternSize.height;
    fsOut << "square_mm" << squareMm;
    fsOut << "used_pairs" << used;
    fsOut << "rms_px" << rms;
    fsOut << "R" << R;
    fsOut << "T" << T;
    fsOut << "E" << E;
    fsOut << "F" << F;

    std::cout << "Used pairs: " << used << "\n";
    std::cout << "Stereo RMS (px): " << rms << "\n";
    std::cout << "Wrote: " << output << "\n";

    return 0;
}
