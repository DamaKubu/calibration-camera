#pragma once

#include <filesystem>
#include <iosfwd>
#include <string>

namespace livemotion {

namespace fs = std::filesystem;

<<<<<<< Updated upstream
=======
struct HsvRange {
    int hMin = 0;
    int sMin = 0;
    int vMin = 0;
    int hMax = 179;
    int sMax = 255;
    int vMax = 255;
};

>>>>>>> Stashed changes
struct AppConfig {
    // Cameras
    fs::path camerasYml = "cameras.yml";
    std::string cam1Key = "cam1";
    std::string cam2Key = "cam2";

    // Calibration files
    fs::path intrinsicsRoot = "data";
    std::string intrinsicsPrefix = "calib_";
    fs::path stereoExtrinsics = "data/pairs_cam1_cam2/stereo_extrinsic.yml";

    // Optional: do a quick stereo calibration live before tracking.
    bool quickCalibrateStereo = false;
    int calibFrames = 25; // number of good chessboard pairs to collect
<<<<<<< Updated upstream
    int patternCols = 9;
    int patternRows = 6;
    double squareMm = 23.6;
    fs::path saveStereoExtrinsics; // if set, writes R/T to this file
    double baselineMm = 0.0; // if > 0, scale translation to this baseline
    bool forceStereo = false; // if true, force R=I and T=[baseline,0,0]
=======
    int patternCols = 8;
    int patternRows = 5;
    double squareMm = 65.0;
    fs::path saveStereoExtrinsics; // if set, writes R/T to this file
>>>>>>> Stashed changes

    // Output
    fs::path outputCsv; // if empty: don't write

<<<<<<< Updated upstream
    // Chessboard tracking
    bool trackBoard = true;
    int boardSkip = 0; // run board detection every N frames (0 = every frame)
    double detScale = 1.0; // downscale factor for detection (e.g. 0.5)

    // Runtime
    bool show = true;
=======
    // Ball detection (HSV)
    HsvRange hsv;
    double minAreaPx = 150.0;
    int morph = 3; // morphological kernel radius

    bool tune = false; // show HSV trackbars + masks

    // Runtime
    bool show = true;
    bool useCuda = true;
>>>>>>> Stashed changes
    int maxFps = 0; // 0 = unlimited

    // Camera request (best-effort)
    unsigned reqWidth = 0;
    unsigned reqHeight = 0;
    unsigned reqFps = 0;
};

void printUsage(std::ostream& os);

// Returns false if help requested.
bool parseArgs(int argc, char** argv, AppConfig& cfg);

int run(const AppConfig& cfg);

} // namespace livemotion
