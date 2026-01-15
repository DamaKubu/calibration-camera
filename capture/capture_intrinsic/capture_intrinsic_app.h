#pragma once

#include <opencv2/opencv.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace capintr {

namespace fs = std::filesystem;

struct AppConfig {
    // --- Camera selection (prefer cameras.yml + Media Foundation) ---
    int cameraIndex = -1;       // OpenCV index fallback (0,1,2,...)
    std::string camKey;         // cam1/cam2/cam3 ... (looked up in cameras.yml)
    std::string camerasYmlPath; // optional override
    std::string symbolicLink;   // Media Foundation symbolic link

    // --- Output ---
    std::string outPath; // optional override (relative to project root)
    int count = 60;
    int pngCompression = 0; // 0 fastest, 9 smallest (lossless)

    // --- Calibration board ---
    cv::Size pattern{8, 5}; // inner corners (W x H)

    // --- OpenCV fallback capture ---
    std::string backend = "any"; // any|msmf|dshow

    // --- Media Foundation open options ---
    unsigned reqWidth = 0;
    unsigned reqHeight = 0;
    unsigned reqFps = 0;

    // --- Auto capture ---
    bool autoEnabled = true;
    int intervalMs = 700;

    // --- Preprocessing (applied before detect/save) ---
    // Matches the behavior of the older capture tool: transforms happen BEFORE chessboard detection and saving.
    int rotateDeg = 0;               // 0|90|180|270
    std::string flipMode = "none";  // none|x|y|xy

    // --- Quality gates (tune these for your setup) ---
    double minBlur = 140.0;      // Variance-of-Laplacian
    double minShiftPx = 14.0;    // mean corner shift vs previous save
    double minBorderPx = 20.0;   // min distance to image border
    double minBoardPx = 20.0;    // average adjacent-corner distance

    // Additional gates to improve calibration quality/stability:
    // - Board should not be too small in the image (otherwise intrinsics become unstable)
    // - Saved views should move around the image (avoid 60 nearly-identical center shots)
    double minBoardAreaRatio = 0.10; // bbox area / image area
    double minCenterShiftNorm = 0.05; // centroid shift vs previous saved (normalized by image diagonal)

    // Anti-wobble: require the board to be stable for a few consecutive frames before saving.
    // This reduces motion blur and corner jitter even if blur metric passes.
    int stableFrames = 4;        // consecutive stable FOUND frames required before a save
    double maxJitterPx = 1.8;    // max mean corner motion between consecutive frames
};

struct ResolvedPaths {
    fs::path projectRoot;
    fs::path camerasYml;
    fs::path outDir;
};

void printUsage(std::ostream& os);

// Parse CLI into cfg. Returns false and sets errorMessage on failure.
bool parseArgs(int argc, char** argv, AppConfig& cfg, std::string& errorMessage);

// Resolve project root, find/load cameras.yml, and resolve output directory.
bool resolvePathsAndCamera(AppConfig& cfg, ResolvedPaths& out, std::string& errorMessage);

// Main application.
int run(const AppConfig& cfg);

} // namespace capintr
