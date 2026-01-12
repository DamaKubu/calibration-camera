#include <opencv2/opencv.hpp>

#include <windows.h>

#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfreadwrite.h>

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")

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
    // If user pastes an escaped string from YAML/JSON, it may contain doubled backslashes.
    // Convert "\\\\?\\usb..." style into "\\?\usb..." style.
    for (;;) {
        const size_t pos = s.find("\\\\\\\\\\\\\\\\");
        if (pos == std::string::npos) break;
        s.replace(pos, 4, "\\\\");
    }
    return s;
}

static bool isAllDigits(const std::string& s) {
    return !s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c) != 0; });
}

static std::wstring toLowerW(std::wstring s) {
    std::transform(s.begin(), s.end(), s.begin(), [](wchar_t c) {
        if (c >= L'A' && c <= L'Z') return (wchar_t)(c - L'A' + L'a');
        return c;
    });
    return s;
}

static std::wstring canonicalizeSymbolicLinkW(const std::wstring& w) {
    // Normalize prefix and collapse doubled backslashes AFTER the leading "\\?\".
    if (w.empty()) return w;

    std::wstring out = w;

    // Reduce "\\\\?\\" (4 slashes) to "\\?\" (2 slashes)
    if (out.rfind(L"\\\\\\\\?\\", 0) == 0) {
        out.erase(0, 2);
    }

    // If it starts with "\\?\", keep prefix; otherwise leave as-is.
    if (out.rfind(L"\\\\?\\", 0) == 0) {
        const std::wstring prefix = out.substr(0, 4); // "\\?\"
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

    // It's OK if this fails; device may not support the requested size/fps.
    return SUCCEEDED(hr);
}

static bool createCameraReaderFromSymbolicLink(
    const std::string& symbolicLink,
    CameraReader& out,
    UINT32 desiredW,
    UINT32 desiredH,
    UINT32 desiredFps) {
    const std::string normalized = normalizeSymbolicLink(symbolicLink);
    const bool byIndex = isAllDigits(normalized);
    const UINT32 targetIndex = byIndex ? (UINT32)std::stoul(normalized) : 0;

    const std::wstring targetWRaw = byIndex ? std::wstring() : utf8ToWide(normalized);
    const std::wstring targetW = byIndex ? std::wstring() : canonicalizeSymbolicLinkW(targetWRaw);
    if (!byIndex && targetW.empty()) {
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
        if (byIndex) {
            if (i == targetIndex) {
                hr = devices[i]->ActivateObject(IID_PPV_ARGS(&source));
                if (FAILED(hr)) {
                    printHresult("ActivateObject(IMFMediaSource)", hr);
                } else {
                    found = true;
                }
                break;
            }
            continue;
        }

        if (SUCCEEDED(devices[i]->GetAllocatedString(
                MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &linkW, &linkLen))) {
            std::wstring linkCanon = canonicalizeSymbolicLinkW(linkW ? std::wstring(linkW) : std::wstring());
            CoTaskMemFree(linkW);
            linkW = nullptr;

            if (!linkCanon.empty() && linkCanon == targetW) {
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

    for (UINT32 i = 0; i < count; ++i) {
        devices[i]->Release();
    }
    CoTaskMemFree(devices);

    if (!found || !source) {
        std::cerr << "ERROR: Could not find a device with that symbolic_link.\n";
        if (byIndex) {
            std::cerr << "Tip: index is 0.." << (count == 0 ? 0 : (count - 1)) << " from camera_true_id.exe listing order.\n";
        } else {
            std::cerr << "Tip: copy/paste the symbolic_link exactly from camera_true_id.exe output.\n";
        }
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
        std::cerr << "ERROR: Could not read frame size from media type\n";
        return false;
    }

    GUID subtype = GUID_NULL;
    hr = type->GetGUID(MF_MT_SUBTYPE, &subtype);
    if (FAILED(hr)) subtype = GUID_NULL;

    LONG stride = 0;
    {
        UINT32 strideU = 0;
        hr = type->GetUINT32(MF_MT_DEFAULT_STRIDE, &strideU);
        if (SUCCEEDED(hr) && strideU != 0) {
            stride = (LONG)strideU;
        }
    }
    if (stride == 0) {
        // Fallback based on common formats.
        // RGB32: 4 bytes/pixel
        // YUY2: 2 bytes/pixel
        // NV12: 1 byte/pixel for luma plane stride
        if (subtype == MFVideoFormat_RGB32) {
            stride = (LONG)(w * 4);
        } else if (subtype == MFVideoFormat_YUY2) {
            stride = (LONG)(w * 2);
        } else {
            stride = (LONG)(w);
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

    if (!sample) {
        return false;
    }

    if (!updateFormatInfo(cam)) {
        sample->Release();
        return false;
    }

    IMFMediaBuffer* buf = nullptr;
    hr = sample->ConvertToContiguousBuffer(&buf);
    if (FAILED(hr)) {
        printHresult("ConvertToContiguousBuffer", hr);
        sample->Release();
        return false;
    }

    BYTE* data = nullptr;
    DWORD maxLen = 0;
    DWORD curLen = 0;
    hr = buf->Lock(&data, &maxLen, &curLen);
    if (FAILED(hr) || !data) {
        printHresult("Lock", hr);
        buf->Release();
        sample->Release();
        return false;
    }

    // Convert based on negotiated subtype.
    if (cam.subtype == MFVideoFormat_RGB32) {
        // MF RGB32 is BGRA byte order.
        cv::Mat bgra((int)cam.height, (int)cam.width, CV_8UC4, data, (size_t)cam.stride);
        cv::cvtColor(bgra, outBgr, cv::COLOR_BGRA2BGR);
    } else if (cam.subtype == MFVideoFormat_YUY2) {
        cv::Mat yuy2((int)cam.height, (int)cam.width, CV_8UC2, data, (size_t)cam.stride);
        cv::cvtColor(yuy2, outBgr, cv::COLOR_YUV2BGR_YUY2);
    } else if (cam.subtype == MFVideoFormat_NV12) {
        // NV12: Y plane (H x W), then interleaved UV plane (H/2 x W/2, 2 channels)
        const int h = (int)cam.height;
        const int w = (int)cam.width;
        const size_t yStride = (size_t)cam.stride;
        unsigned char* yPtr = data;
        unsigned char* uvPtr = data + yStride * (size_t)h;
        cv::Mat y(h, w, CV_8UC1, yPtr, yStride);
        cv::Mat uv(h / 2, w / 2, CV_8UC2, uvPtr, yStride);
        cv::cvtColorTwoPlane(y, uv, outBgr, cv::COLOR_YUV2BGR_NV12);
    } else {
        // Unknown: try treating as RGB32 to at least show something.
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
        << "dual_camera_preview_by_id\n"
        << "\nUsage:\n"
        << "  dual_camera_preview_by_id.exe --cam1 <symbolic_link> --cam2 <symbolic_link> [--width 1280 --height 720 --fps 30]\n"
        << "  dual_camera_preview_by_id.exe --cam1 0 --cam2 1  (open by enumeration index)\n"
        << "  dual_camera_preview_by_id.exe --list\n"
        << "\nNotes:\n"
        << "- Use camera_true_id.exe to list symbolic_link values.\n"
        << "- Press ESC to quit.\n";
}

static int listDevices() {
    IMFAttributes* attr = nullptr;
    HRESULT hr = MFCreateAttributes(&attr, 1);
    if (FAILED(hr)) {
        printHresult("MFCreateAttributes", hr);
        return 1;
    }
    hr = attr->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
    if (FAILED(hr)) {
        printHresult("SetGUID(SOURCE_TYPE)", hr);
        attr->Release();
        return 1;
    }

    IMFActivate** devices = nullptr;
    UINT32 count = 0;
    hr = MFEnumDeviceSources(attr, &devices, &count);
    attr->Release();
    if (FAILED(hr)) {
        printHresult("MFEnumDeviceSources", hr);
        return 1;
    }

    std::cout << "Found " << count << " device(s)\n\n";
    for (UINT32 i = 0; i < count; ++i) {
        wchar_t* nameW = nullptr;
        UINT32 nameLen = 0;
        wchar_t* linkW = nullptr;
        UINT32 linkLen = 0;

        std::cout << "[" << i << "] ";
        if (SUCCEEDED(devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &nameW, &nameLen))) {
            std::wcout << nameW;
            CoTaskMemFree(nameW);
        } else {
            std::cout << "(unknown name)";
        }
        std::cout << "\n";

        if (SUCCEEDED(devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &linkW, &linkLen))) {
            std::wcout << L"    symbolic_link: " << linkW << L"\n";
            CoTaskMemFree(linkW);
        } else {
            std::cout << "    symbolic_link: (unavailable)\n";
        }

        std::cout << "\n";
    }

    for (UINT32 i = 0; i < count; ++i) devices[i]->Release();
    CoTaskMemFree(devices);
    return 0;
}

int main(int argc, char** argv) {
    std::string cam1Link;
    std::string cam2Link;
    UINT32 width = 0;
    UINT32 height = 0;
    UINT32 fps = 0;
    bool doList = false;

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
        } else if (a == "--list") {
            doList = true;
        } else if (a == "--cam1") {
            cam1Link = need("--cam1");
        } else if (a == "--cam2") {
            cam2Link = need("--cam2");
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

    if (doList) {
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
        const int rc = listDevices();
        MFShutdown();
        CoUninitialize();
        return rc;
    }

    if (cam1Link.empty() || cam2Link.empty()) {
        std::cerr << "ERROR: --cam1 and --cam2 are required\n";
        printUsage();
        return 2;
    }

    cam1Link = normalizeSymbolicLink(cam1Link);
    cam2Link = normalizeSymbolicLink(cam2Link);

    std::cout << "cam1 symbolic_link: " << cam1Link << "\n";
    std::cout << "cam2 symbolic_link: " << cam2Link << "\n";

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

    if (!createCameraReaderFromSymbolicLink(cam1Link, cam1, width, height, fps)) {
        std::cerr << "ERROR: Failed to open cam1 by symbolic_link\n";
        MFShutdown();
        CoUninitialize();
        return 1;
    }

    if (!createCameraReaderFromSymbolicLink(cam2Link, cam2, width, height, fps)) {
        std::cerr << "ERROR: Failed to open cam2 by symbolic_link\n";
        MFShutdown();
        CoUninitialize();
        return 1;
    }

    cv::namedWindow("cam1", cv::WINDOW_NORMAL);
    cv::namedWindow("cam2", cv::WINDOW_NORMAL);

    int noFrameCount = 0;

    while (true) {
        cv::Mat b1, b2;
        const bool ok1 = readFrameBGR(cam1, b1);
        const bool ok2 = readFrameBGR(cam2, b2);

        if (ok1 && !b1.empty()) cv::imshow("cam1", b1);
        if (ok2 && !b2.empty()) cv::imshow("cam2", b2);

        if ((!ok1 || b1.empty()) && (!ok2 || b2.empty())) {
            ++noFrameCount;
            if (noFrameCount == 60) {
                std::cout << "Still waiting for frames... (if windows are black, try --cam1 0 --cam2 1 or close other camera apps)\n";
            }
        } else {
            noFrameCount = 0;
        }

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 27) break;
    }

    cv::destroyAllWindows();

    MFShutdown();
    CoUninitialize();
    return 0;
}
