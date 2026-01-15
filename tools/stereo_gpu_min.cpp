// FLOWCHART
// [Start]
//   -> load intrinsics/extrinsics from YAML
//   -> open cam0/cam1 (VideoCapture)
//   -> loop:
//        grab frames
//        upload to GPU
//        gray -> absdiff with previous -> threshold
//        download mask -> largest blob center per cam
//        undistort to normalized rays
//        triangulate -> print XYZ
//        draw 3D view (projected) + trail
//   -> [End]
//
// Build note: add this file to your CMake target if desired.

#include <opencv2/opencv.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <iostream>
#include <deque>

static bool loadIntr(const std::string& p, cv::Mat& K, cv::Mat& D) {
    cv::FileStorage fs(p, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;
    return !K.empty() && !D.empty();
}

static bool loadExt(const std::string& p, cv::Mat& R, cv::Mat& T) {
    cv::FileStorage fs(p, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["R"] >> R; fs["T"] >> T;
    return !R.empty() && !T.empty();
}

static bool blobCenter(const cv::Mat& m, cv::Point2f& c, double minArea) {
    std::vector<std::vector<cv::Point>> cs; cv::findContours(m, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double best=0; cv::Point2f out;
    for (auto& k: cs){
        double a=cv::contourArea(k);
        if(a<minArea) continue;
        if(a>best){ auto mm=cv::moments(k); if(mm.m00>1e-6){ best=a; out={float(mm.m10/mm.m00),float(mm.m01/mm.m00)};}}
    }
    if (best<=0) return false; c=out; return true;
}

static bool triangulate(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Mat& R, const cv::Mat& T, cv::Point3d& X) {
    cv::Mat P1 = cv::Mat::eye(3,4,CV_64F), P2 = cv::Mat::zeros(3,4,CV_64F);
    cv::Mat R64,T64; R.convertTo(R64,CV_64F); T.convertTo(T64,CV_64F); R64.copyTo(P2(cv::Rect(0,0,3,3))); T64.copyTo(P2(cv::Rect(3,0,1,3)));
    cv::Mat pts1(2,1,CV_64F), pts2(2,1,CV_64F); pts1.at<double>(0)=p1.x; pts1.at<double>(1)=p1.y; pts2.at<double>(0)=p2.x; pts2.at<double>(1)=p2.y;
    cv::Mat Xh; cv::triangulatePoints(P1,P2,pts1,pts2,Xh); double w=Xh.at<double>(3); if (std::abs(w)<1e-12) return false;
    X.x=Xh.at<double>(0)/w; X.y=Xh.at<double>(1)/w; X.z=Xh.at<double>(2)/w; return true;
}

static void draw3D(cv::Mat& view, const cv::Point3d& X, std::deque<cv::Point3d>& trail) {
    const int w = view.cols, h = view.rows;
    const cv::Point2f c(w * 0.5f, h * 0.5f);
    const double z = std::max(1.0, X.z);
    const double f = 800.0; // virtual focal length
    auto proj = [&](const cv::Point3d& p) -> cv::Point2f {
        double zz = std::max(1.0, p.z);
        return cv::Point2f((float)(c.x + f * p.x / zz), (float)(c.y - f * p.y / zz));
    };

    if (X.z > 0 && X.z < 20000) {
        trail.push_back(X);
        if (trail.size() > 200) trail.pop_front();
    }

    view.setTo(cv::Scalar(10, 10, 10));
    // axes
    cv::line(view, proj({0,0,0}), proj({500,0,0}), cv::Scalar(0,0,255), 2);
    cv::line(view, proj({0,0,0}), proj({0,500,0}), cv::Scalar(0,255,0), 2);
    cv::line(view, proj({0,0,0}), proj({0,0,500}), cv::Scalar(255,0,0), 2);

    // trail
    for (size_t i = 1; i < trail.size(); ++i) {
        cv::line(view, proj(trail[i-1]), proj(trail[i]), cv::Scalar(255, 255, 0), 1);
    }
    if (!trail.empty()) {
        cv::circle(view, proj(trail.back()), 5, cv::Scalar(0, 255, 255), -1);
    }
}

int main() {
    cv::Mat K1,D1,K2,D2,R,T;
    if (!loadIntr("data/calib_cam1/intrinsic.yml", K1,D1) || !loadIntr("data/calib_cam2/intrinsic.yml", K2,D2) || !loadExt("reports/stereo_live.yml", R,T)) {
        std::cerr << "Failed to load calibration\n"; return 1;
    }
    cv::VideoCapture c0(0), c1(1);
    if (!c0.isOpened() || !c1.isOpened()) { std::cerr << "Failed to open cameras\n"; return 1; }

    cv::cuda::GpuMat g0, g1, g0g, g1g, g0p, g1p, gd0, gd1, gm0, gm1; 
    cv::Mat view3d(480, 640, CV_8UC3);
    std::deque<cv::Point3d> trail;
    cv::namedWindow("view3d", cv::WINDOW_NORMAL);

    const double MIN_AREA = 800.0;   // px
    const double MIN_DISP = 2.0;     // px
    const double MAX_DISP = 800.0;   // px
    const double Z_MIN = 100.0;      // mm
    const double Z_MAX = 10000.0;    // mm

    for (;;) {
        cv::Mat f0,f1; if(!c0.read(f0)||!c1.read(f1)) continue;
        g0.upload(f0); g1.upload(f1);
        cv::cuda::cvtColor(g0, g0g, cv::COLOR_BGR2GRAY);
        cv::cuda::cvtColor(g1, g1g, cv::COLOR_BGR2GRAY);
        if (g0p.empty()) { g0p=g0g.clone(); g1p=g1g.clone(); continue; }
        cv::cuda::absdiff(g0g, g0p, gd0); cv::cuda::absdiff(g1g, g1p, gd1);
        cv::cuda::threshold(gd0, gm0, 20, 255, cv::THRESH_BINARY);
        cv::cuda::threshold(gd1, gm1, 20, 255, cv::THRESH_BINARY);
        g0p = g0g.clone(); g1p = g1g.clone();
        cv::Mat m0,m1; gm0.download(m0); gm1.download(m1);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
        cv::morphologyEx(m0, m0, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(m0, m0, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(m1, m1, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(m1, m1, cv::MORPH_CLOSE, kernel);

        cv::Point2f cA,cB; if (!blobCenter(m0,cA,MIN_AREA) || !blobCenter(m1,cB,MIN_AREA)) continue;
        double disp = std::abs(cA.x - cB.x);
        if (disp < MIN_DISP || disp > MAX_DISP) continue;
        std::vector<cv::Point2f> s1{cA}, s2{cB}, u1,u2; cv::undistortPoints(s1,u1,K1,D1); cv::undistortPoints(s2,u2,K2,D2);
        cv::Point3d X; 
        if (triangulate(u1[0], u2[0], R, T, X)) {
            if (X.z < Z_MIN || X.z > Z_MAX) continue;
            std::cout << "XYZ(mm): " << X.x << "," << X.y << "," << X.z << "\n";
            draw3D(view3d, X, trail);
            cv::imshow("view3d", view3d);
        }
        if (cv::waitKey(1) == 27) break;
    }
    return 0;
}
