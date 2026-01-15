#pragma once

#include <filesystem>
#include <opencv2/core.hpp>
#include <string>

namespace intrin {

struct AppConfig {
    // Input
    std::string cameraId;               // if set: imagesDir defaults to data/<cameraId>
    std::filesystem::path imagesDir;    // can be relative (resolved against data root)
    std::string extLower = "png";

    // Chessboard
    cv::Size pattern{8, 5};
    double squareMm = 65.0;

    // Model
    // "standard" -> Brown-Conrady 5 params: k1 k2 p1 p2 k3
    // "fisheye"  -> OpenCV fisheye 4 params
    std::string model = "standard";

    // Data quality / pruning
    int minDetections = 10;
    double maxPerImageErrorPx = 1.2;
    int maxRemove = 15;                 // will be clamped internally for stability
    int minDatasetForPrune = 20;        // do not prune below this size
    double minBoardAreaRatio = 0.10;    // reject detections where board bbox area is too small
    double minRmsImprovement = 1e-3;    // stop pruning when RMS improvement stalls

    // Output
    std::filesystem::path output;       // default: <imagesDir>/intrinsic.yml

    // Diagnostics
    std::filesystem::path reportCsv;    // if set: write per-image detection stats

    // Verbosity
    bool verbose = true;
};

void printUsage();

// Returns false if help was printed and program should exit(0).
// Throws std::runtime_error for bad args.
bool parseArgs(int argc, char** argv, AppConfig& cfg);

int run(const AppConfig& cfg);

} // namespace intrin
