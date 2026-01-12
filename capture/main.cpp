#include <opencv2/opencv.hpp>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    int cameraIndex = 0;
    std::string cameraId = "cam0";
    int count = 60;
    int intervalMs = 0;
    int pngCompression = 0;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                std::exit(2);
            }
            return argv[++i];
        };

        if (a == "--camera") {
            cameraIndex = std::stoi(need("--camera"));
        } else if (a == "--camera-id") {
            cameraId = need("--camera-id");
        } else if (a == "--count") {
            count = std::stoi(need("--count"));
        } else if (a == "--interval-ms") {
            intervalMs = std::stoi(need("--interval-ms"));
        } else if (a == "--png-compression") {
            pngCompression = std::stoi(need("--png-compression"));
            if (pngCompression < 0) pngCompression = 0;
            if (pngCompression > 9) pngCompression = 9;
        } else if (a == "--help" || a == "-h") {
            std::cout
                << "capture\\n"
                << "Usage:\\n"
                << "  capture.exe --camera 0 --camera-id cam0 --count 60 [--interval-ms 0]\\n"
                << "\\n"
                << "Saves images to: data/<camera_id>/img_*.png\\n"
                << "Controls: [Space]=save  [Esc]=quit\\n";
            return 0;
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            return 2;
        }
    }

    if (count <= 0) {
        std::cerr << "--count must be > 0\n";
        return 2;
    }

    const fs::path outDir = fs::path("data") / cameraId;
    fs::create_directories(outDir);

    int nextIndex = 0;
    if (fs::exists(outDir) && fs::is_directory(outDir)) {
        for (const auto& e : fs::directory_iterator(outDir)) {
            if (!e.is_regular_file()) continue;
            const auto p = e.path();
            const auto name = p.filename().string();
            if (name.rfind("img_", 0) != 0) continue;
            if (p.extension().string() != ".png") continue;

            const auto stem = p.stem().string(); // img_####
            if (stem.size() < 5) continue;
            const auto numStr = stem.substr(4);
            try {
                int n = std::stoi(numStr);
                if (n >= nextIndex) nextIndex = n + 1;
            } catch (...) {
            }
        }
    }

    cv::VideoCapture cap(cameraIndex);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Cannot open camera " << cameraIndex << "\n";
        return 1;
    }

    std::cout << "Saving to: " << outDir.string() << "\\n";
    std::cout << "Press [Space] to save, [Esc] to quit.\\n";
    if (intervalMs > 0) {
        std::cout << "Auto-save every " << intervalMs << " ms.\\n";
    }

    const std::string win = "capture";
    cv::namedWindow(win, cv::WINDOW_NORMAL);

    int saved = 0;
    auto lastSave = std::chrono::steady_clock::now();

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        cv::imshow(win, frame);

        bool doSave = false;
        if (intervalMs > 0) {
            auto now = std::chrono::steady_clock::now();
            int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSave).count();
            if (elapsed >= intervalMs) {
                doSave = true;
                lastSave = now;
            }
        }

        int key = cv::waitKey(1) & 0xFF;
        if (key == 27) break;
        if (key == 32) doSave = true;

        if (doSave) {
            std::ostringstream ss;
            ss << "img_" << std::setw(4) << std::setfill('0') << nextIndex << ".png";
            fs::path outPath = outDir / ss.str();

            std::vector<int> params;
            params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            params.push_back(pngCompression);

            if (!cv::imwrite(outPath.string(), frame, params)) {
                std::cerr << "ERROR: Failed to write " << outPath.string() << "\n";
                return 1;
            }

            ++nextIndex;
            ++saved;
            std::cout << "Saved " << outPath.string() << " (" << saved << "/" << count << ")\n";

            if (saved >= count) break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
