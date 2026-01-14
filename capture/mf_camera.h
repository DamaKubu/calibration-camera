#pragma once

#include <opencv2/core.hpp>

#include <filesystem>
#include <string>

namespace mfcam {

struct OpenOptions {
    unsigned width = 0;
    unsigned height = 0;
    unsigned fps = 0;
};

class Camera {
public:
    Camera();
    ~Camera();

    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    bool openSymbolicLink(const std::string& symbolicLink, const OpenOptions& opts = {});
    bool readBgr(cv::Mat& outBgr);

    void close();

private:
    struct Impl;
    Impl* impl_;
};

// Load a symbolic_link value from a cameras.yml file.
// camKey examples: "cam1", "cam2".
bool loadSymbolicLinkFromCamerasYml(const std::filesystem::path& ymlPath, const std::string& camKey, std::string& outLink);

// Try to locate a file by walking up parent directories.
std::filesystem::path findUpwardsForFile(const std::filesystem::path& startDir, const std::filesystem::path& filename, int maxHops = 8);

} // namespace mfcam
