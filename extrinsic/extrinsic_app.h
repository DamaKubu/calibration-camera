#pragma once

#include <opencv2/core.hpp>

#include <filesystem>
#include <iosfwd>
#include <string>

namespace capext {

namespace fs = std::filesystem;

struct AppConfig {
    // --- Defaults (keep these obvious & tweakable) ---
    std::string cameraId = "cam0";
    cv::Size pattern{8, 5};
    double squareMm = 65.0;

    // If set, runs multi-camera session mode.
    std::string sessionDir; // e.g. data/extrinsic_multi/session_0
    std::string refCam;     // e.g. cam1

    // Multi-camera settings
    std::string method = "pnp"; // pnp | stereo | refine
    double maxPnpReprojPx = 2.0;
    int distN = 8;
    std::string intrinsicsRoot = "data";
    std::string intrinsicsPrefix = "calib_";
};

void printUsage(std::ostream& os);

// Parse CLI into cfg. Returns false on error; if errorMessage == "help" treat as help-request.
bool parseArgs(int argc, char** argv, AppConfig& cfg, std::string& errorMessage);

// Run application.
int run(const AppConfig& cfg);

} // namespace capext
