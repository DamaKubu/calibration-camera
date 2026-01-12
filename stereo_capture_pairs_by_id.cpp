#include <opencv2/opencv.hpp>

#include <windows.h>

#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfreadwrite.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")

namespace fs = std::filesystem;

static void printHresult(const char* what, HRESULT hr) {
    std::cerr << what << " failed (HRESULT=0x" << std::hex << std::setw(8) << std::setfill('0')
              << (unsigned)hr << std::dec << ")\n";
}

static std::wstring utf8ToWide(const std::string& s) {
    if (s.empty()) return {};
    int len = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
    if (len <= 0) return {};
    std::wstring out;
    out.resize((size_t)len);
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, &out[0], len);
    if (!out.empty() && out.back() == L'\0') out.pop_back();
    return out;
}

static std::string normalizeSymbolicLink(std::string s) {
    // Collapse "\\\\" sequences into "\\" (helps when pasting escaped strings).
    for (;;) {
        const size_t pos = s.find("\\\\\\\\\\\\\\\\");
        if (pos == std::string::npos) break;
        s.replace(pos, 4, "\\\\");
    }
    return s;
}

struct MfReleaser {
    void operator()(IUnknown* p) const {
        if (p) p->Release();
    }
};

template <class T>
using com_ptr = std::unique_ptr<T, MfReleaser>;

template <class T>
static com_ptr<T> adopt(T* p) {
    return com_ptr<T>(p);
}

struct CameraReader {
    com_ptr<IMFMediaSource> source;
    com_ptr<IMFSourceReader> reader;

    UINT32 width = 0;
    UINT32 height = 0;
    LONG stride = 0;
    GUID subtype = GUID_NULL;
    bool formatKnown = false;
};

static std::wstring toLowerW(std::wstring s) {
    std::transform(s.begin(), s.end(), s.begin(), [](wchar_t c) {
        if (c >= L'A' && c <= L'Z') return (wchar_t)(c - L'A' + L'a');
        return c;
    });
    return s;
}

static std::wstring canonicalizeSymbolicLinkW(const std::wstring& w) {
    if (w.empty()) return w;
    std::wstring out = w;

    // Reduce "\\\\?\\" (4 slashes) to "\\?\\" (2 slashes)
    if (out.rfind(L"\\\\\\\\?\\", 0) == 0) {
        out.erase(0, 2);
    }

    if (out.rfind(L"\\\\?\\", 0) == 0) {
        const std::wstring prefix = out.substr(0, 4);
        std::wstring rest = out.substr(4);
        for (;;) {
            const size_t pos = rest.find(L"\\\\");
            if (pos == std::wstring::npos) break;
            rest.replace(pos, 2, L"\\");
        }
        out = prefix + rest;
    }

    return toLowerW(out);
}

static bool setReaderOutputType(IMFSourceReader* reader, UINT32 desiredW, UINT32 desiredH, UINT32 desiredFps) {
    IMFMediaType* type = nullptr;
    HRESULT hr = MFCreateMediaType(&type);
    if (FAILED(hr)) {
        printHresult("MFCreateMediaType", hr);
        return false;
    }

    type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB32);

    if (desiredW > 0 && desiredH > 0) {
        MFSetAttributeSize(type, MF_MT_FRAME_SIZE, desiredW, desiredH);
    }
    if (desiredFps > 0) {
        MFSetAttributeRatio(type, MF_MT_FRAME_RATE, desiredFps, 1);
    }

    hr = reader->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, type);
    type->Release();

    return SUCCEEDED(hr);
}

static bool openBySymbolicLink(const std::string& symbolicLink, CameraReader& out, UINT32 desiredW, UINT32 desiredH, UINT32 desiredFps) {
    const std::wstring targetW = canonicalizeSymbolicLinkW(utf8ToWide(normalizeSymbolicLink(symbolicLink)));
    if (targetW.empty()) {
        std::cerr << "ERROR: empty symbolic_link\n";
        return false;
    }

    IMFAttributes* attr = nullptr;
    HRESULT hr = MFCreateAttributes(&attr, 1);
    if (FAILED(hr)) {
        printHresult("MFCreateAttributes", hr);
        return false;
    }
    hr = attr->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
    if (FAILED(hr)) {
        printHresult("SetGUID(SOURCE_TYPE)", hr);
        attr->Release();
        return false;
    }

    IMFActivate** devices = nullptr;
    UINT32 count = 0;
    hr = MFEnumDeviceSources(attr, &devices, &count);
    attr->Release();
    if (FAILED(hr)) {
        printHresult("MFEnumDeviceSources", hr);
        return false;
    }

    IMFMediaSource* source = nullptr;
    bool found = false;

    for (UINT32 i = 0; i < count; ++i) {
        wchar_t* linkW = nullptr;
        UINT32 linkLen = 0;
        if (SUCCEEDED(devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &linkW, &linkLen))) {
            std::wstring canon = canonicalizeSymbolicLinkW(linkW ? std::wstring(linkW) : std::wstring());
            CoTaskMemFree(linkW);

            if (!canon.empty() && canon == targetW) {
                hr = devices[i]->ActivateObject(IID_PPV_ARGS(&source));
                if (FAILED(hr)) {
                    printHresult("ActivateObject(IMFMediaSource)", hr);
                } else {
                    found = true;
                }
                break;
            }
        }
    }

    for (UINT32 i = 0; i < count; ++i) devices[i]->Release();
    CoTaskMemFree(devices);

    if (!found || !source) {
        std::cerr << "ERROR: Could not find device with that symbolic_link\n";
        return false;
    }

    IMFSourceReader* reader = nullptr;
    hr = MFCreateSourceReaderFromMediaSource(source, nullptr, &reader);
    if (FAILED(hr)) {
        printHresult("MFCreateSourceReaderFromMediaSource", hr);
        source->Release();
        return false;
    }

    setReaderOutputType(reader, desiredW, desiredH, desiredFps);

    out.source = adopt(source);
    out.reader = adopt(reader);
    return true;
}

static bool updateFormatInfo(CameraReader& cam) {
    if (cam.formatKnown) return true;

    IMFMediaType* type = nullptr;
    HRESULT hr = cam.reader->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, &type);
    if (FAILED(hr)) {
        printHresult("GetCurrentMediaType", hr);
        return false;
    }

    UINT32 w = 0, h = 0;
    hr = MFGetAttributeSize(type, MF_MT_FRAME_SIZE, &w, &h);
    if (FAILED(hr) || w == 0 || h == 0) {
        type->Release();
        return false;
    }

    GUID subtype = GUID_NULL;
    hr = type->GetGUID(MF_MT_SUBTYPE, &subtype);
    if (FAILED(hr)) subtype = GUID_NULL;

    LONG stride = 0;
    {
        UINT32 strideU = 0;
        hr = type->GetUINT32(MF_MT_DEFAULT_STRIDE, &strideU);
        if (SUCCEEDED(hr) && strideU != 0) stride = (LONG)strideU;
    }
    if (stride == 0) {
        if (subtype == MFVideoFormat_RGB32) {
            stride = (LONG)(w * 4);
        } else if (subtype == MFVideoFormat_YUY2) {
            stride = (LONG)(w * 2);
        } else {
            stride = (LONG)w;
        }
    }

    type->Release();

    cam.width = w;
    cam.height = h;
    cam.stride = stride;
    cam.subtype = subtype;
    cam.formatKnown = true;
    return true;
}

static bool readFrameBGR(CameraReader& cam, cv::Mat& outBgr) {
    DWORD streamIndex = 0;
    DWORD flags = 0;
    LONGLONG ts = 0;
    IMFSample* sample = nullptr;

    HRESULT hr = cam.reader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, &streamIndex, &flags, &ts, &sample);
    if (FAILED(hr)) {
        printHresult("ReadSample", hr);
        return false;
    }

    if (flags & MF_SOURCE_READERF_STREAMTICK) {
        if (sample) sample->Release();
        return false;
    }

    if (!sample) return false;

    if (!updateFormatInfo(cam)) {
        sample->Release();
        return false;
    }

    IMFMediaBuffer* buf = nullptr;
    hr = sample->ConvertToContiguousBuffer(&buf);
    if (FAILED(hr)) {
        sample->Release();
        return false;
    }

    BYTE* data = nullptr;
    DWORD maxLen = 0;
    DWORD curLen = 0;
    hr = buf->Lock(&data, &maxLen, &curLen);
    if (FAILED(hr) || !data) {
        buf->Release();
        sample->Release();
        return false;
    }

    if (cam.subtype == MFVideoFormat_RGB32) {
        cv::Mat bgra((int)cam.height, (int)cam.width, CV_8UC4, data, (size_t)cam.stride);
        cv::cvtColor(bgra, outBgr, cv::COLOR_BGRA2BGR);
    } else if (cam.subtype == MFVideoFormat_YUY2) {
        cv::Mat yuy2((int)cam.height, (int)cam.width, CV_8UC2, data, (size_t)cam.stride);
        cv::cvtColor(yuy2, outBgr, cv::COLOR_YUV2BGR_YUY2);
    } else if (cam.subtype == MFVideoFormat_NV12) {
        const int h = (int)cam.height;
        const int w = (int)cam.width;
        const size_t yStride = (size_t)cam.stride;
        unsigned char* yPtr = data;
        unsigned char* uvPtr = data + yStride * (size_t)h;
        cv::Mat y(h, w, CV_8UC1, yPtr, yStride);
        cv::Mat uv(h / 2, w / 2, CV_8UC2, uvPtr, yStride);
        cv::cvtColorTwoPlane(y, uv, outBgr, cv::COLOR_YUV2BGR_NV12);
    } else {
        cv::Mat bgra((int)cam.height, (int)cam.width, CV_8UC4, data, (size_t)cam.stride);
        cv::cvtColor(bgra, outBgr, cv::COLOR_BGRA2BGR);
    }

    buf->Unlock();
    buf->Release();
    sample->Release();
    return true;
}

static void printUsage() {
    std::cout
        << "stereo_capture_pairs_by_id\n"
        << "\nUsage:\n"
        << "  stereo_capture_pairs_by_id.exe --cam1 <symbolic_link> --cam2 <symbolic_link> --out-dir data/pairs_cam1_cam2 [--count 30]\n"
        << "\nOptions:\n"
        << "  --pattern WxH    Chessboard inner corners (default 8x5)\n"
        << "  --auto           Auto-save when both cameras detect chessboard\n"
        << "  --stable N       Require N consecutive frames with detection in BOTH cams (default 5)\n"
        << "  --min-ms MS      Minimum milliseconds between auto-saves (default 350)\n"
        << "  --width N        Request width (optional)\n"
        << "  --height N       Request height (optional)\n"
        << "  --fps N          Request fps (optional)\n"
        << "\nControls:\n"
        << "  Space : save one synchronized pair (manual)\n"
        << "  Esc   : quit\n";
}

static bool parsePattern(const std::string& s, cv::Size& pattern) {
    const auto x = s.find('x');
    if (x == std::string::npos) return false;
    try {
        int w = std::stoi(s.substr(0, x));
        int h = std::stoi(s.substr(x + 1));
        if (w <= 0 || h <= 0) return false;
        pattern = cv::Size(w, h);
        return true;
    } catch (...) {
        return false;
    }
}

static bool detectChessboard(const cv::Mat& bgr, const cv::Size& pattern, std::vector<cv::Point2f>& corners) {
    corners.clear();
    if (bgr.empty()) return false;

    cv::Mat gray;
    if (bgr.channels() == 1) {
        gray = bgr;
    } else {
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    }

    bool found = false;
#if (CV_VERSION_MAJOR >= 4)
    // SB is generally more robust (and slower) than the classic detector.
    found = cv::findChessboardCornersSB(gray, pattern, corners,
                                        cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE);
    if (!found) {
        corners.clear();
        found = cv::findChessboardCorners(gray, pattern, corners,
                                          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    }
#else
    found = cv::findChessboardCorners(gray, pattern, corners,
                                      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
#endif

    if (!found) {
        corners.clear();
        return false;
    }

    // Refine for nicer visualization and better saved data.
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 1e-3));
    return true;
}

static void drawStatusOverlay(cv::Mat& bgr, const cv::Size& pattern, bool found, const std::vector<cv::Point2f>& corners,
                              const std::string& textLine1, const std::string& textLine2) {
    if (bgr.empty()) return;

    if (found) {
        cv::drawChessboardCorners(bgr, pattern, corners, true);
    }

    const int font = cv::FONT_HERSHEY_SIMPLEX;
    const double scale = 0.7;
    const int thickness = 2;

    const cv::Scalar color = found ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 220);
    cv::putText(bgr, textLine1, cv::Point(10, 28), font, scale, color, thickness, cv::LINE_AA);
    cv::putText(bgr, textLine2, cv::Point(10, 55), font, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
}

int main(int argc, char** argv) {
    std::string cam1Link;
    std::string cam2Link;
    fs::path outDir = fs::path("data") / "pairs_cam1_cam2";
    int count = 30;
    UINT32 width = 0;
    UINT32 height = 0;
    UINT32 fps = 0;
    cv::Size pattern(8, 5);
    bool autoMode = false;
    int stable = 5;
    int minMs = 350;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                std::exit(2);
            }
            return argv[++i];
        };

        if (a == "--help" || a == "-h") {
            printUsage();
            return 0;
        } else if (a == "--cam1") {
            cam1Link = need("--cam1");
        } else if (a == "--cam2") {
            cam2Link = need("--cam2");
        } else if (a == "--out-dir") {
            outDir = need("--out-dir");
        } else if (a == "--count") {
            count = std::stoi(need("--count"));
        } else if (a == "--pattern") {
            cv::Size p;
            if (!parsePattern(need("--pattern"), p)) {
                std::cerr << "Bad --pattern; expected WxH like 8x5\n";
                return 2;
            }
            pattern = p;
        } else if (a == "--auto") {
            autoMode = true;
        } else if (a == "--stable") {
            stable = std::stoi(need("--stable"));
        } else if (a == "--min-ms") {
            minMs = std::stoi(need("--min-ms"));
        } else if (a == "--width") {
            width = (UINT32)std::stoul(need("--width"));
        } else if (a == "--height") {
            height = (UINT32)std::stoul(need("--height"));
        } else if (a == "--fps") {
            fps = (UINT32)std::stoul(need("--fps"));
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            printUsage();
            return 2;
        }
    }

    if (cam1Link.empty() || cam2Link.empty()) {
        std::cerr << "ERROR: --cam1 and --cam2 are required\n";
        printUsage();
        return 2;
    }
    if (count <= 0) {
        std::cerr << "ERROR: --count must be > 0\n";
        return 2;
    }
    if (stable <= 0) {
        std::cerr << "ERROR: --stable must be > 0\n";
        return 2;
    }
    if (minMs < 0) {
        std::cerr << "ERROR: --min-ms must be >= 0\n";
        return 2;
    }

    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(hr)) {
        printHresult("CoInitializeEx", hr);
        return 1;
    }

    hr = MFStartup(MF_VERSION);
    if (FAILED(hr)) {
        printHresult("MFStartup", hr);
        CoUninitialize();
        return 1;
    }

    CameraReader cam1;
    CameraReader cam2;

    if (!openBySymbolicLink(cam1Link, cam1, width, height, fps)) {
        std::cerr << "ERROR: Failed to open cam1\n";
        MFShutdown();
        CoUninitialize();
        return 1;
    }
    if (!openBySymbolicLink(cam2Link, cam2, width, height, fps)) {
        std::cerr << "ERROR: Failed to open cam2\n";
        MFShutdown();
        CoUninitialize();
        return 1;
    }

    fs::create_directories(outDir);

    cv::namedWindow("cam1", cv::WINDOW_NORMAL);
    cv::namedWindow("cam2", cv::WINDOW_NORMAL);

    int saved = 0;
    int nextIndex = 0;

    int bothFoundStreak = 0;
    auto lastAutoSave = std::chrono::steady_clock::now() - std::chrono::milliseconds(10'000);

    std::cout << "Saving pairs to: " << fs::absolute(outDir).string() << "\n";
    std::cout << "Pattern: " << pattern.width << "x" << pattern.height << " (inner corners)\n";
    if (autoMode) {
        std::cout << "AUTO mode ON: need both detections for " << stable << " consecutive frames; min " << minMs << " ms between saves\n";
    }
    std::cout << "Press Space to save pair, Esc to quit.\n";

    while (true) {
        cv::Mat f1, f2;
        const bool ok1 = readFrameBGR(cam1, f1);
        const bool ok2 = readFrameBGR(cam2, f2);

        std::vector<cv::Point2f> c1, c2;
        bool found1 = false;
        bool found2 = false;

        cv::Mat show1 = f1.empty() ? f1 : f1.clone();
        cv::Mat show2 = f2.empty() ? f2 : f2.clone();

        if (ok1 && !f1.empty()) {
            found1 = detectChessboard(f1, pattern, c1);
            std::ostringstream s1, s2;
            s1 << "cam1: " << (found1 ? "FOUND" : "NO") << " | saved " << saved << "/" << count;
            s2 << "streak " << bothFoundStreak << "/" << stable << (autoMode ? " | AUTO" : " | manual");
            drawStatusOverlay(show1, pattern, found1, c1, s1.str(), s2.str());
            cv::imshow("cam1", show1);
        }
        if (ok2 && !f2.empty()) {
            found2 = detectChessboard(f2, pattern, c2);
            std::ostringstream s1, s2;
            s1 << "cam2: " << (found2 ? "FOUND" : "NO") << " | saved " << saved << "/" << count;
            s2 << "streak " << bothFoundStreak << "/" << stable << (autoMode ? " | AUTO" : " | manual");
            drawStatusOverlay(show2, pattern, found2, c2, s1.str(), s2.str());
            cv::imshow("cam2", show2);
        }

        if (found1 && found2) {
            bothFoundStreak++;
        } else {
            bothFoundStreak = 0;
        }

        auto trySavePair = [&](const cv::Mat& a, const cv::Mat& b) -> bool {
            if (a.empty() || b.empty()) {
                std::cout << "Skipping save (one frame missing)\n";
                return false;
            }

            std::ostringstream n;
            n << std::setw(4) << std::setfill('0') << nextIndex;

            const fs::path p1 = outDir / (std::string("cam1_") + n.str() + ".png");
            const fs::path p2 = outDir / (std::string("cam2_") + n.str() + ".png");

            if (!cv::imwrite(p1.string(), a) || !cv::imwrite(p2.string(), b)) {
                std::cerr << "ERROR: Failed to write pair\n";
                return false;
            }

            ++nextIndex;
            ++saved;
            std::cout << "Saved pair " << saved << "/" << count << " (" << p1.filename().string() << ", "
                      << p2.filename().string() << ")\n";
            return true;
        };

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 27) break;

        if (key == 32) {
            if (!trySavePair(f1, f2)) {
                continue;
            }
            bothFoundStreak = 0;
            lastAutoSave = std::chrono::steady_clock::now();
            if (saved >= count) break;
        }

        if (autoMode && saved < count) {
            const auto now = std::chrono::steady_clock::now();
            const auto msSince = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - lastAutoSave).count();
            if (bothFoundStreak >= stable && msSince >= minMs) {
                if (trySavePair(f1, f2)) {
                    bothFoundStreak = 0;
                    lastAutoSave = now;
                    if (saved >= count) break;
                }
            }
        }
    }

    cv::destroyAllWindows();

    MFShutdown();
    CoUninitialize();
    return 0;
}
