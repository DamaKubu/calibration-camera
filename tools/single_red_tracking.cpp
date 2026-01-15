// FLOWCHART
// [Start]
//   -> open cam1
//   -> loop:
//        grab frame
//        convert to HSV
//        threshold red (two ranges)
//        find largest blob
//        estimate XY (meters) from A4 size
//        estimate Z from apparent width
//        show preview + trace window
//   -> [End]

#include <opencv2/opencv.hpp>
#include <iostream>
#include <deque>

int main() {
    cv::VideoCapture cam(0);
    if (!cam.isOpened()) { std::cerr << "Failed to open cam1\n"; return 1; }

    cv::namedWindow("cam1", cv::WINDOW_NORMAL);
    cv::namedWindow("trace", cv::WINDOW_NORMAL);
    cv::Mat trace(480, 640, CV_8UC3);
    std::deque<cv::Point2f> trail;

    const double A4_W_M = 0.210; // meters
    const double A4_H_M = 0.297; // meters
    const double FOCAL_PX = 1000.0; // approximate; replace with real fx if available

    for (;;) {
        cv::Mat frame; if (!cam.read(frame)) continue;
        cv::Mat hsv; cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat m1, m2, mask;
        cv::inRange(hsv, cv::Scalar(0, 80, 80), cv::Scalar(10, 255, 255), m1);
        cv::inRange(hsv, cv::Scalar(160, 80, 80), cv::Scalar(179, 255, 255), m2);
        cv::bitwise_or(m1, m2, mask);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        double bestArea = 0.0; cv::Point2f center; cv::Rect bestBox;
        for (auto& c : contours) {
            double a = cv::contourArea(c);
            if (a > bestArea) {
                cv::Moments m = cv::moments(c);
                if (m.m00 > 1e-6) { bestArea = a; center = {(float)(m.m10/m.m00), (float)(m.m01/m.m00)}; bestBox = cv::boundingRect(c); }
            }
        }

        if (bestArea > 0.0) {
            double wpx = std::max(1.0, (double)bestBox.width);
            double hpx = std::max(1.0, (double)bestBox.height);
            double z = (FOCAL_PX * A4_W_M) / wpx; // meters
            double scale = A4_W_M / wpx;
            cv::Point2f origin((float)(frame.cols * 0.5), (float)(frame.rows * 0.5));
            double x_m = (center.x - origin.x) * scale;
            double y_m = -(center.y - origin.y) * scale;

            std::cout << "XY(m): " << x_m << "," << y_m << "  Z(m): " << z << "\n";
            cv::circle(frame, center, 6, cv::Scalar(0,255,0), 2);
            cv::rectangle(frame, bestBox, cv::Scalar(0,255,255), 1);

            // trace window
            trace.setTo(cv::Scalar(10,10,10));
            cv::Point2f tcenter(trace.cols * 0.5f, trace.rows * 0.5f);
            cv::line(trace, {0,(int)tcenter.y}, {trace.cols,(int)tcenter.y}, cv::Scalar(40,40,40), 1);
            cv::line(trace, {(int)tcenter.x,0}, {(int)tcenter.x,trace.rows}, cv::Scalar(40,40,40), 1);

            cv::Point2f pt2d((float)(tcenter.x + x_m * 300.0), (float)(tcenter.y - y_m * 300.0)); // 1m = 300px
            trail.push_back(pt2d);
            if (trail.size() > 200) trail.pop_front();
            for (size_t i = 1; i < trail.size(); ++i) {
                cv::line(trace, trail[i-1], trail[i], cv::Scalar(0,255,255), 1);
            }
            cv::circle(trace, pt2d, 4, cv::Scalar(0,255,0), -1);
            cv::imshow("trace", trace);
        }

        cv::imshow("cam1", frame);
        if ((cv::waitKey(1) & 0xFF) == 27) break;
    }
    return 0;
}
