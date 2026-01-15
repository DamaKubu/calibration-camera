#include "extrinsic_app.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace capext {

using std::cerr;
using std::cout;
using std::string;
namespace fs = std::filesystem;

namespace {

fs::path findUpwardsForFile(fs::path start, const string& filename) {
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
    return {};
}

fs::path defaultProjectRoot() {
    const fs::path cmake = findUpwardsForFile(fs::current_path(), "CMakeLists.txt");
    return cmake.empty() ? fs::current_path() : cmake.parent_path();
}

bool readIntrinsics(const fs::path& intrPath, cv::Mat& K, cv::Mat& D) {
    K.release();
    D.release();
    cv::FileStorage fs(intrPath.string(), cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;
    fs.release();
    if (K.empty() || K.rows != 3 || K.cols != 3) return false;
    return !D.empty();
}

std::vector<cv::Point3f> makeObjectPoints(const cv::Size& pattern, double squareMm) {
    std::vector<cv::Point3f> obj;
    obj.reserve((size_t)pattern.width * (size_t)pattern.height);
    for (int y = 0; y < pattern.height; ++y) {
        for (int x = 0; x < pattern.width; ++x) {
            obj.push_back(cv::Point3f((float)(x * squareMm), (float)(y * squareMm), 0.0f));
        }
    }
    return obj;
}

bool detectCorners(const cv::Mat& bgr, const cv::Size& pattern, std::vector<cv::Point2f>& corners) {
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

    bool found = cv::findChessboardCornersSB(gray, pattern, corners, cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ACCURACY);
    if (!found) {
        found = cv::findChessboardCorners(gray, pattern, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    }
    if (!found) return false;

    cv::cornerSubPix(gray, corners, cv::Size(15, 15), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 200, 1e-12));
    return true;
}

bool parsePattern(const string& s, cv::Size& out) {
    const auto x = s.find('x');
    if (x == string::npos) return false;
    const int w = std::stoi(s.substr(0, x));
    const int h = std::stoi(s.substr(x + 1));
    if (w <= 0 || h <= 0) return false;
    out = cv::Size(w, h);
    return true;
}

bool parseShotIndex(const string& filename, int& idxOut) {
    if (filename.rfind("shot_", 0) != 0) return false;
    if (filename.size() < 9) return false;
    const string num = filename.substr(5, 4);
    for (char c : num) {
        if (c < '0' || c > '9') return false;
    }
    idxOut = std::stoi(num);
    return true;
}

struct Quat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

Quat quatFromR(const cv::Mat& R) {
    const double m00 = R.at<double>(0, 0), m01 = R.at<double>(0, 1), m02 = R.at<double>(0, 2);
    const double m10 = R.at<double>(1, 0), m11 = R.at<double>(1, 1), m12 = R.at<double>(1, 2);
    const double m20 = R.at<double>(2, 0), m21 = R.at<double>(2, 1), m22 = R.at<double>(2, 2);

    Quat q;
    const double tr = m00 + m11 + m22;
    if (tr > 0.0) {
        const double S = std::sqrt(tr + 1.0) * 2.0;
        q.w = 0.25 * S;
        q.x = (m21 - m12) / S;
        q.y = (m02 - m20) / S;
        q.z = (m10 - m01) / S;
    } else if ((m00 > m11) && (m00 > m22)) {
        const double S = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q.w = (m21 - m12) / S;
        q.x = 0.25 * S;
        q.y = (m01 + m10) / S;
        q.z = (m02 + m20) / S;
    } else if (m11 > m22) {
        const double S = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q.w = (m02 - m20) / S;
        q.x = (m01 + m10) / S;
        q.y = 0.25 * S;
        q.z = (m12 + m21) / S;
    } else {
        const double S = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q.w = (m10 - m01) / S;
        q.x = (m02 + m20) / S;
        q.y = (m12 + m21) / S;
        q.z = 0.25 * S;
    }

    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n > 0.0) {
        q.w /= n;
        q.x /= n;
        q.y /= n;
        q.z /= n;
    }
    return q;
}

cv::Mat RFromQuat(const Quat& qIn) {
    Quat q = qIn;
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n > 0.0) {
        q.w /= n;
        q.x /= n;
        q.y /= n;
        q.z /= n;
    }

    const double ww = q.w * q.w;
    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;

    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    R.at<double>(0, 0) = ww + xx - yy - zz;
    R.at<double>(0, 1) = 2.0 * (q.x * q.y - q.w * q.z);
    R.at<double>(0, 2) = 2.0 * (q.x * q.z + q.w * q.y);

    R.at<double>(1, 0) = 2.0 * (q.x * q.y + q.w * q.z);
    R.at<double>(1, 1) = ww - xx + yy - zz;
    R.at<double>(1, 2) = 2.0 * (q.y * q.z - q.w * q.x);

    R.at<double>(2, 0) = 2.0 * (q.x * q.z - q.w * q.y);
    R.at<double>(2, 1) = 2.0 * (q.y * q.z + q.w * q.x);
    R.at<double>(2, 2) = ww - xx - yy + zz;
    return R;
}

double rotationAngleDeg(const cv::Mat& R) {
    const double tr = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
    double c = (tr - 1.0) * 0.5;
    c = std::max(-1.0, std::min(1.0, c));
    return std::acos(c) * 180.0 / CV_PI;
}

double reprojErrorPx(const std::vector<cv::Point3f>& obj, const std::vector<cv::Point2f>& img, const cv::Mat& rvec, const cv::Mat& tvec,
                     const cv::Mat& K, const cv::Mat& D) {
    std::vector<cv::Point2f> proj;
    cv::projectPoints(obj, rvec, tvec, K, D, proj);
    if (proj.size() != img.size() || img.empty()) return 1e9;
    double sum = 0.0;
    for (size_t i = 0; i < img.size(); ++i) {
        const cv::Point2f d = proj[i] - img[i];
        sum += d.x * d.x + d.y * d.y;
    }
    return std::sqrt(sum / (double)img.size());
}

bool solvePnPBest(const std::vector<cv::Point3f>& obj, const std::vector<cv::Point2f>& img, const cv::Mat& K, const cv::Mat& D,
                 cv::Mat& outRvec, cv::Mat& outTvec, double& outErrPx) {
    outRvec.release();
    outTvec.release();
    outErrPx = 1e9;

    std::vector<cv::Mat> rvecs, tvecs;
    try {
        cv::solvePnPGeneric(obj, img, K, D, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    } catch (...) {
        rvecs.clear();
        tvecs.clear();
    }

    if (!rvecs.empty() && rvecs.size() == tvecs.size()) {
        int best = -1;
        double bestErr = 1e9;
        int bestPos = -1;
        double bestPosErr = 1e9;

        for (int i = 0; i < (int)rvecs.size(); ++i) {
            cv::Mat r = rvecs[(size_t)i];
            cv::Mat t = tvecs[(size_t)i];
            r.convertTo(r, CV_64F);
            t.convertTo(t, CV_64F);
            const double err = reprojErrorPx(obj, img, r, t, K, D);

            if (err < bestErr) {
                bestErr = err;
                best = i;
            }
            if (t.rows == 3 && t.cols == 1 && t.at<double>(2, 0) > 0.0) {
                if (err < bestPosErr) {
                    bestPosErr = err;
                    bestPos = i;
                }
            }
        }

        const int pick = (bestPos >= 0) ? bestPos : best;
        if (pick >= 0) {
            outRvec = rvecs[(size_t)pick];
            outTvec = tvecs[(size_t)pick];
            outRvec.convertTo(outRvec, CV_64F);
            outTvec.convertTo(outTvec, CV_64F);
            outErrPx = reprojErrorPx(obj, img, outRvec, outTvec, K, D);
            return true;
        }
    }

    cv::Mat r, t;
    if (!cv::solvePnP(obj, img, K, D, r, t, false, cv::SOLVEPNP_ITERATIVE)) return false;
    r.convertTo(r, CV_64F);
    t.convertTo(t, CV_64F);
    outRvec = r;
    outTvec = t;
    outErrPx = reprojErrorPx(obj, img, outRvec, outTvec, K, D);
    return true;
}

int runSessionMode(const AppConfig& cfg, const fs::path& projectRoot, const fs::path& intrinsicsRootAbs) {
    auto clipDist = [&](const cv::Mat& D) -> cv::Mat {
        if (cfg.distN == 0) return cv::Mat::zeros(0, 1, CV_64F);
        if (D.empty()) return D;
        cv::Mat d;
        D.convertTo(d, CV_64F);
        const int total = (int)d.total();
        const int n = std::min(cfg.distN, total);
        cv::Mat flat = d.reshape(1, total);
        return flat.rowRange(0, n).clone();
    };

    fs::path sessionPath(cfg.sessionDir);
    if (!sessionPath.is_absolute()) sessionPath = projectRoot / sessionPath;
    sessionPath = fs::absolute(sessionPath);

    const fs::path sessionYml = sessionPath / "session.yml";
    if (!fs::exists(sessionYml)) {
        cerr << "ERROR: Missing session.yml: " << sessionYml.string() << "\n";
        return 1;
    }

    cv::FileStorage fsSession(sessionYml.string(), cv::FileStorage::READ);
    if (!fsSession.isOpened()) {
        cerr << "ERROR: Cannot open session.yml: " << sessionYml.string() << "\n";
        return 1;
    }

    cv::Size pattern = cfg.pattern;
    double squareMm = cfg.squareMm;

    int patternCols = 0, patternRows = 0;
    double squareMmFromSession = 0.0;
    fsSession["pattern_cols"] >> patternCols;
    fsSession["pattern_rows"] >> patternRows;
    fsSession["square_mm"] >> squareMmFromSession;
    if (patternCols > 0 && patternRows > 0) pattern = cv::Size(patternCols, patternRows);
    if (squareMmFromSession > 0.0) squareMm = squareMmFromSession;

    std::vector<string> cams;
    {
        cv::FileNode n = fsSession["cameras"];
        if (n.type() != cv::FileNode::SEQ) {
            cerr << "ERROR: session.yml missing 'cameras' list\n";
            return 1;
        }
        for (auto it = n.begin(); it != n.end(); ++it) {
            string key;
            (*it)["key"] >> key;
            if (!key.empty()) cams.push_back(key);
        }
    }
    fsSession.release();

    if (cams.size() < 2) {
        cerr << "ERROR: session has <2 cameras\n";
        return 1;
    }

    string refCam = cfg.refCam.empty() ? cams.front() : cfg.refCam;

    int refIdx = -1;
    for (int i = 0; i < (int)cams.size(); ++i) {
        if (cams[(size_t)i] == refCam) {
            refIdx = i;
            break;
        }
    }
    if (refIdx < 0) {
        cerr << "ERROR: --ref " << refCam << " not found in session cameras\n";
        return 1;
    }

    struct CamCalib {
        cv::Mat K;
        cv::Mat D;
    };
    std::vector<CamCalib> calib(cams.size());
    for (size_t i = 0; i < cams.size(); ++i) {
        const fs::path intrPath = intrinsicsRootAbs / (cfg.intrinsicsPrefix + cams[i]) / "intrinsic.yml";
        if (!readIntrinsics(intrPath, calib[i].K, calib[i].D)) {
            cerr << "ERROR: Missing/invalid intrinsics for " << cams[i] << ": " << intrPath.string() << "\n";
            return 1;
        }
    }

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
        cerr << "ERROR: No shot_XXXX_*.png found in " << sessionPath.string() << "\n";
        return 1;
    }

    const std::vector<cv::Point3f> obj = makeObjectPoints(pattern, squareMm);

    struct Shot {
        int idx = -1;
        std::vector<cv::Size> imageSize;
        std::vector<bool> found;
        std::vector<std::vector<cv::Point2f>> corners;
    };

    std::vector<Shot> shots;
    shots.reserve(shotIdxs.size());

    std::vector<int> foundCounts(cams.size(), 0);
    int shotCounter = 0;
    const int totalShots = (int)shotIdxs.size();

    for (int idx : shotIdxs) {
        ++shotCounter;
        if (shotCounter == 1 || (shotCounter % 5) == 0 || shotCounter == totalShots) {
            cout << "Detecting corners: " << shotCounter << "/" << totalShots << " (shot_" << std::setw(4) << std::setfill('0') << idx << ")\n";
        }

        Shot s;
        s.idx = idx;
        s.imageSize.assign(cams.size(), cv::Size());
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
            s.imageSize[ci] = img.size();

            std::vector<cv::Point2f> pts;
            if (detectCorners(img, pattern, pts)) {
                s.found[ci] = true;
                s.corners[ci] = std::move(pts);
                foundCounts[ci]++;
            }
        }
        if (!anyRead) continue;
        shots.push_back(std::move(s));
    }

    cout << "Detected corners per camera:\n";
    for (size_t i = 0; i < cams.size(); ++i) {
        cout << "  " << cams[i] << ": " << foundCounts[i] << "/" << shots.size() << "\n";
    }

    const int minViews = 20;
    const cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 1000, 1e-12);

    const fs::path outPath = sessionPath / "extrinsics.yml";
    cv::FileStorage out(outPath.string(), cv::FileStorage::WRITE);
    out << "session" << sessionPath.string();
    out << "ref_cam" << refCam;
    out << "method" << cfg.method;
    out << "max_pnp_reproj_px" << cfg.maxPnpReprojPx;
    out << "dist_n" << cfg.distN;
    out << "pattern_cols" << pattern.width;
    out << "pattern_rows" << pattern.height;
    out << "square_mm" << squareMm;
    out << "intrinsics_root" << intrinsicsRootAbs.string();
    out << "intrinsics_prefix" << cfg.intrinsicsPrefix;

    out << "cameras" << "[";
    for (const auto& c : cams) out << c;
    out << "]";

    out << "pairs" << "[";

    for (size_t ci = 0; ci < cams.size(); ++ci) {
        if ((int)ci == refIdx) continue;

        std::vector<std::vector<cv::Point3f>> objectPoints;
        std::vector<std::vector<cv::Point2f>> imgRef;
        std::vector<std::vector<cv::Point2f>> imgOther;
        std::vector<cv::Size> refSizes;
        std::vector<cv::Size> otherSizes;

        for (const auto& s : shots) {
            if (!s.found[(size_t)refIdx] || !s.found[ci]) continue;
            objectPoints.push_back(obj);
            imgRef.push_back(s.corners[(size_t)refIdx]);
            imgOther.push_back(s.corners[ci]);
            refSizes.push_back(s.imageSize[(size_t)refIdx]);
            otherSizes.push_back(s.imageSize[ci]);
        }

        if ((int)objectPoints.size() < minViews) {
            cerr << "ERROR: Not enough views for pair " << refCam << " -> " << cams[ci] << " (have " << objectPoints.size()
                 << ", need >= " << minViews << ")\n";
            return 1;
        }

        const cv::Mat D1 = clipDist(calib[(size_t)refIdx].D);
        const cv::Mat D2 = clipDist(calib[ci].D);

        cv::Mat R, T, E, F;
        double rms = -1.0;
        double transStdMm = 0.0;
        double rotStdDeg = 0.0;

        auto sizesMatch = [&]() -> bool {
            if (refSizes.size() != otherSizes.size() || refSizes.empty()) return false;
            for (size_t i = 0; i < refSizes.size(); ++i) {
                if (refSizes[i] != otherSizes[i]) return false;
            }
            return true;
        };

        const bool canStereo = sizesMatch();
        if ((cfg.method == "stereo" || cfg.method == "refine") && !canStereo) {
            cout << "WARN: image sizes differ for pair " << refCam << " -> " << cams[ci] << " (e.g. " << refSizes.front().width << "x"
                 << refSizes.front().height << " vs " << otherSizes.front().width << "x" << otherSizes.front().height
                 << "). Falling back to PnP method.\n";
        }

        const string effectiveMethod = ((cfg.method == "stereo" || cfg.method == "refine") && canStereo) ? cfg.method : "pnp";

        if (effectiveMethod == "stereo" || effectiveMethod == "refine") {
            int stereoFlags = cv::CALIB_FIX_INTRINSIC;

            if (effectiveMethod == "refine") {
                std::vector<cv::Mat> Rs;
                std::vector<cv::Mat> Ts;
                Rs.reserve(objectPoints.size());
                Ts.reserve(objectPoints.size());

                for (size_t vi = 0; vi < objectPoints.size(); ++vi) {
                    cv::Mat rvec1, tvec1, rvec2, tvec2;
                    double err1 = 1e9, err2 = 1e9;
                    const bool ok1 = solvePnPBest(objectPoints[vi], imgRef[vi], calib[(size_t)refIdx].K, D1, rvec1, tvec1, err1);
                    const bool ok2 = solvePnPBest(objectPoints[vi], imgOther[vi], calib[ci].K, D2, rvec2, tvec2, err2);
                    if (!ok1 || !ok2) continue;
                    if (err1 > cfg.maxPnpReprojPx || err2 > cfg.maxPnpReprojPx) continue;

                    cv::Mat R1, R2;
                    cv::Rodrigues(rvec1, R1);
                    cv::Rodrigues(rvec2, R2);
                    R1.convertTo(R1, CV_64F);
                    R2.convertTo(R2, CV_64F);
                    tvec1.convertTo(tvec1, CV_64F);
                    tvec2.convertTo(tvec2, CV_64F);

                    cv::Mat R21 = R2 * R1.t();
                    cv::Mat t21 = tvec2 - (R21 * tvec1);
                    Rs.push_back(R21);
                    Ts.push_back(t21);
                }

                if ((int)Rs.size() >= minViews) {
                    Quat qsum{0, 0, 0, 0};
                    for (const auto& Ri : Rs) {
                        Quat qi = quatFromR(Ri);
                        if (qi.w < 0.0) {
                            qi.w = -qi.w;
                            qi.x = -qi.x;
                            qi.y = -qi.y;
                            qi.z = -qi.z;
                        }
                        qsum.w += qi.w;
                        qsum.x += qi.x;
                        qsum.y += qi.y;
                        qsum.z += qi.z;
                    }
                    R = RFromQuat(qsum);
                    T = cv::Mat::zeros(3, 1, CV_64F);
                    for (const auto& Ti : Ts) T += Ti;
                    T /= (double)Ts.size();
                    stereoFlags |= cv::CALIB_USE_EXTRINSIC_GUESS;
                }
            }

            const cv::Size imageSize = refSizes.front();
            rms = cv::stereoCalibrate(objectPoints, imgRef, imgOther, calib[(size_t)refIdx].K, D1, calib[ci].K, D2, imageSize, R, T, E, F,
                                      stereoFlags, criteria);
        } else {
            std::vector<cv::Mat> Rs;
            std::vector<cv::Mat> Ts;
            Rs.reserve(objectPoints.size());
            Ts.reserve(objectPoints.size());
            int rejectedHighErr = 0;

            for (size_t vi = 0; vi < objectPoints.size(); ++vi) {
                cv::Mat rvec1, tvec1, rvec2, tvec2;
                double err1 = 1e9, err2 = 1e9;
                const bool ok1 = solvePnPBest(objectPoints[vi], imgRef[vi], calib[(size_t)refIdx].K, D1, rvec1, tvec1, err1);
                const bool ok2 = solvePnPBest(objectPoints[vi], imgOther[vi], calib[ci].K, D2, rvec2, tvec2, err2);
                if (!ok1 || !ok2) continue;

                if (err1 > cfg.maxPnpReprojPx || err2 > cfg.maxPnpReprojPx) {
                    ++rejectedHighErr;
                    continue;
                }

                cv::Mat R1, R2;
                cv::Rodrigues(rvec1, R1);
                cv::Rodrigues(rvec2, R2);
                R1.convertTo(R1, CV_64F);
                R2.convertTo(R2, CV_64F);
                tvec1.convertTo(tvec1, CV_64F);
                tvec2.convertTo(tvec2, CV_64F);

                cv::Mat R21 = R2 * R1.t();
                cv::Mat t21 = tvec2 - (R21 * tvec1);
                Rs.push_back(R21);
                Ts.push_back(t21);
            }

            if ((int)Rs.size() < minViews) {
                cerr << "ERROR: Not enough valid solvePnP views for pair " << refCam << " -> " << cams[ci] << " (have " << Rs.size()
                     << ", rejected_high_err=" << rejectedHighErr << ", need >= " << minViews << ")\n";
                return 1;
            }

            Quat qsum{0, 0, 0, 0};
            for (const auto& Ri : Rs) {
                Quat qi = quatFromR(Ri);
                if (qi.w < 0.0) {
                    qi.w = -qi.w;
                    qi.x = -qi.x;
                    qi.y = -qi.y;
                    qi.z = -qi.z;
                }
                qsum.w += qi.w;
                qsum.x += qi.x;
                qsum.y += qi.y;
                qsum.z += qi.z;
            }
            R = RFromQuat(qsum);

            T = cv::Mat::zeros(3, 1, CV_64F);
            for (const auto& Ti : Ts) T += Ti;
            T /= (double)Ts.size();

            {
                double sumT = 0.0, sumT2 = 0.0;
                double sumA = 0.0, sumA2 = 0.0;
                for (size_t vi = 0; vi < Ts.size(); ++vi) {
                    const double tn = cv::norm(Ts[vi] - T);
                    sumT += tn;
                    sumT2 += tn * tn;
                    const cv::Mat dR = Rs[vi] * R.t();
                    const double a = rotationAngleDeg(dR);
                    sumA += a;
                    sumA2 += a * a;
                }
                const double n = (double)Ts.size();
                const double meanT = sumT / n;
                const double meanA = sumA / n;
                transStdMm = std::sqrt(std::max(0.0, (sumT2 / n) - meanT * meanT));
                rotStdDeg = std::sqrt(std::max(0.0, (sumA2 / n) - meanA * meanA));
            }
        }

        out << "{";
        out << "cam1" << refCam;
        out << "cam2" << cams[ci];
        out << "pair_method" << effectiveMethod;
        out << "views" << (int)objectPoints.size();
        if (effectiveMethod == "stereo" || effectiveMethod == "refine") {
            out << "rms" << rms;
        } else {
            out << "trans_std_mm" << transStdMm;
            out << "rot_std_deg" << rotStdDeg;
        }
        out << "R" << R;
        out << "T" << T;
        out << "baseline_mm" << cv::norm(T);
        if (effectiveMethod == "stereo" || effectiveMethod == "refine") {
            out << "E" << E;
            out << "F" << F;
        }
        out << "}";

        if (effectiveMethod == "stereo" || effectiveMethod == "refine") {
            cout << "Pair " << refCam << " -> " << cams[ci] << ": views=" << objectPoints.size() << " RMS=" << rms
                 << " baseline_mm=" << cv::norm(T) << "\n";
        } else {
            cout << "Pair " << refCam << " -> " << cams[ci] << ": views=" << objectPoints.size() << " baseline_mm=" << cv::norm(T)
                 << " trans_std_mm=" << transStdMm << " rot_std_deg=" << rotStdDeg << "\n";
        }
    }

    out << "]";
    out.release();
    cout << "Wrote: " << outPath.string() << "\n";
    return 0;
}

int runSingleCameraMode(const AppConfig& cfg, const fs::path& projectRoot) {
    const fs::path dir = projectRoot / "data" / cfg.cameraId;
    const fs::path intrPath = dir / "intrinsic.yml";
    const fs::path outPath = dir / "extrinsic.yml";

    cv::Mat K, D;
    if (!readIntrinsics(intrPath, K, D)) {
        cerr << "ERROR: Invalid intrinsics in " << intrPath.string() << "\n";
        return 1;
    }

    if (!fs::exists(dir) || !fs::is_directory(dir)) {
        cerr << "ERROR: Missing input directory: " << dir.string() << "\n";
        return 1;
    }

    std::vector<fs::path> files;
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
        cerr << "ERROR: No images found in " << dir.string() << "\n";
        return 1;
    }

    const std::vector<cv::Point3f> obj = makeObjectPoints(cfg.pattern, cfg.squareMm);

    cv::Mat rvec, tvec;
    string usedImage;

    for (const auto& p : files) {
        cv::Mat img = cv::imread(p.string(), cv::IMREAD_COLOR);
        if (img.empty()) continue;

        std::vector<cv::Point2f> corners;
        if (!detectCorners(img, cfg.pattern, corners)) continue;

        if (!cv::solvePnP(obj, corners, K, D, rvec, tvec)) continue;
        usedImage = p.filename().string();
        break;
    }

    if (usedImage.empty() || rvec.empty() || tvec.empty()) {
        cerr << "ERROR: Could not estimate pose from any image.\n";
        return 1;
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);

    cv::FileStorage out(outPath.string(), cv::FileStorage::WRITE);
    if (!out.isOpened()) {
        cerr << "ERROR: Cannot open for writing: " << outPath.string() << "\n";
        return 1;
    }

    out << "camera_id" << cfg.cameraId;
    out << "image_used" << usedImage;
    out << "pattern_size_w" << cfg.pattern.width;
    out << "pattern_size_h" << cfg.pattern.height;
    out << "square_size_mm" << cfg.squareMm;
    out << "rvec" << rvec;
    out << "tvec" << tvec;
    out << "R" << R;
    out.release();
    return 0;
}

} // namespace

void printUsage(std::ostream& os) {
    os << "extrinsic\n"
          "Usage:\n"
          "  (single camera pose)\n"
          "    extrinsic.exe --camera-id cam0 [--pattern 8x5] [--square-mm 65]\n\n"
          "  (multi-camera session extrinsics)\n"
          "    extrinsic.exe --session data/extrinsic_multi/session_0 --ref cam1 [--method pnp|stereo|refine]\n"
          "    extrinsic.exe --session data/extrinsic_multi/session_0 --ref cam1 --method refine --dist-n 8 --max-pnp-reproj 2.0\n\n"
          "Single camera mode:\n"
          "  Reads images from: data/<camera_id>/img_*.png\n"
          "  Reads intrinsics from: data/<camera_id>/intrinsic.yml\n"
          "  Writes extrinsics to: data/<camera_id>/extrinsic.yml\n\n"
          "Multi-camera mode:\n"
          "  Reads: <session>/session.yml and shot_XXXX_<cam>.png files.\n"
          "  Reads intrinsics from: <intrinsics_root>/<intrinsics_prefix><cam>/intrinsic.yml\n"
          "  Writes: <session>/extrinsics.yml (cam_ref -> cam_i)\n"
          "  Notes: 'pnp' works even if camera image sizes differ; 'stereo/refine' requires matching sizes for the pair.\n";
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
            if (a == "--session") {
                cfg.sessionDir = need(i, "--session");
                continue;
            }
            if (a == "--ref") {
                cfg.refCam = need(i, "--ref");
                continue;
            }
            if (a == "--method") {
                cfg.method = need(i, "--method");
                std::transform(cfg.method.begin(), cfg.method.end(), cfg.method.begin(), [](unsigned char c) { return (char)std::tolower(c); });
                if (cfg.method != "pnp" && cfg.method != "stereo" && cfg.method != "refine") {
                    errorMessage = "--method must be 'pnp', 'stereo', or 'refine'";
                    return false;
                }
                continue;
            }
            if (a == "--max-pnp-reproj") {
                cfg.maxPnpReprojPx = std::stod(need(i, "--max-pnp-reproj"));
                continue;
            }
            if (a == "--dist-n") {
                cfg.distN = std::stoi(need(i, "--dist-n"));
                if (cfg.distN < 0) cfg.distN = 0;
                continue;
            }
            if (a == "--intrinsics-root") {
                cfg.intrinsicsRoot = need(i, "--intrinsics-root");
                continue;
            }
            if (a == "--intrinsics-prefix") {
                cfg.intrinsicsPrefix = need(i, "--intrinsics-prefix");
                continue;
            }
            if (a == "--camera-id") {
                cfg.cameraId = need(i, "--camera-id");
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
            if (a == "--square-mm") {
                cfg.squareMm = std::stod(need(i, "--square-mm"));
                continue;
            }

            errorMessage = "Unknown arg: " + a;
            return false;
        }
    } catch (const std::exception& e) {
        errorMessage = e.what();
        return false;
    }

    return true;
}

int run(const AppConfig& cfg) {
    const fs::path projectRoot = defaultProjectRoot();

    fs::path intrinsicsRootAbs(cfg.intrinsicsRoot);
    if (!intrinsicsRootAbs.is_absolute()) intrinsicsRootAbs = projectRoot / intrinsicsRootAbs;

    if (!cfg.sessionDir.empty()) {
        return runSessionMode(cfg, projectRoot, intrinsicsRootAbs);
    }
    return runSingleCameraMode(cfg, projectRoot);
}

} // namespace capext
