#include "mf_camera.h"

#include <opencv2/imgproc.hpp>

#include <windows.h>

#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfreadwrite.h>

#include <algorithm>
#include <iostream>
#include <memory>

#include <yaml-cpp/yaml.h>

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "ole32.lib")

namespace fs = std::filesystem;

namespace {

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
static std::wstring stripGlobalSuffixW(const std::wstring& s) {
    if (s.size() >= 7) {
        const std::wstring suf1 = L"\\global";
        const std::wstring suf2 = L"/global";
        if (s.size() >= suf1.size() && s.compare(s.size() - suf1.size(), suf1.size(), suf1) == 0) {
            std::wstring result = s;
            result.erase(result.size() - suf1.size());
            return result;
        }
        if (s.size() >= suf2.size() && s.compare(s.size() - suf2.size(), suf2.size(), suf2) == 0) {
            std::wstring result = s;
            result.erase(result.size() - suf2.size());
            return result;
        }
    }
    return s;
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

static bool ensureMfStarted() {
    static bool started = false;
    if (started) return true;

    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(hr) && hr != RPC_E_CHANGED_MODE) {
        return false;
    }

    hr = MFStartup(MF_VERSION);
    if (FAILED(hr)) {
        return false;
    }

    started = true;
    return true;
}

static bool setReaderOutputType(IMFSourceReader* reader, unsigned desiredW, unsigned desiredH, unsigned desiredFps) {
    IMFMediaType* type = nullptr;
    HRESULT hr = MFCreateMediaType(&type);
    if (FAILED(hr)) return false;

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

static bool openBySymbolicLink(const std::string& symbolicLink, com_ptr<IMFMediaSource>& outSource, com_ptr<IMFSourceReader>& outReader,
                               const mfcam::OpenOptions& opts) {
    const std::wstring targetW = canonicalizeSymbolicLinkW(utf8ToWide(normalizeSymbolicLink(symbolicLink)));
    if (targetW.empty()) return false;
    const std::wstring targetNoGlobal = stripGlobalSuffixW(targetW);

    IMFAttributes* attr = nullptr;
    HRESULT hr = MFCreateAttributes(&attr, 1);
    if (FAILED(hr)) return false;

    hr = attr->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
    if (FAILED(hr)) {
        attr->Release();
        return false;
    }

    IMFActivate** devices = nullptr;
    UINT32 count = 0;
    hr = MFEnumDeviceSources(attr, &devices, &count);
    attr->Release();
    if (FAILED(hr)) return false;

    IMFMediaSource* source = nullptr;
    std::wstring lastSeen;
    for (UINT32 i = 0; i < count; ++i) {
        wchar_t* linkW = nullptr;
        UINT32 linkLen = 0;
        if (SUCCEEDED(devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &linkW, &linkLen))) {
            std::wstring canon = canonicalizeSymbolicLinkW(linkW ? std::wstring(linkW) : std::wstring());
            CoTaskMemFree(linkW);
            if (canon.empty()) continue;
            lastSeen = canon;

            const std::wstring canonNoGlobal = stripGlobalSuffixW(canon);
            const bool matchExact = (canon == targetW);
            const bool matchNoGlobal = (!targetNoGlobal.empty() && canonNoGlobal == targetNoGlobal);
            const bool matchPrefixA = (!targetNoGlobal.empty() && canon.rfind(targetNoGlobal, 0) == 0);
            const bool matchPrefixB = (!canonNoGlobal.empty() && targetW.rfind(canonNoGlobal, 0) == 0);

            if (matchExact || matchNoGlobal || matchPrefixA || matchPrefixB) {
                hr = devices[i]->ActivateObject(IID_PPV_ARGS(&source));
                if (SUCCEEDED(hr) && source) {
                    break;
                }
                source = nullptr;
            }
        }
    }

    auto cleanupDevices = [&]() {
        for (UINT32 j = 0; j < count; ++j) devices[j]->Release();
        CoTaskMemFree(devices);
        devices = nullptr;
        count = 0;
    };

    if (!source) {
        std::wcerr << L"MF open failed: no device matched symbolic_link\n";
        std::wcerr << L"  target: " << targetW << L"\n";
        if (!targetNoGlobal.empty() && targetNoGlobal != targetW) {
            std::wcerr << L"  target(no-global): " << targetNoGlobal << L"\n";
        }

        // Enumerate a few devices to help update cameras.yml.
        if (devices && count > 0) {
            std::wcerr << L"  available devices (first 8):\n";
            for (UINT32 i = 0; i < count && i < 8; ++i) {
                wchar_t* linkW = nullptr;
                UINT32 linkLen = 0;
                if (SUCCEEDED(devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &linkW, &linkLen))) {
                    std::wstring canon = canonicalizeSymbolicLinkW(linkW ? std::wstring(linkW) : std::wstring());
                    CoTaskMemFree(linkW);
                    if (!canon.empty()) {
                        std::wcerr << L"    - " << canon << L"\n";
                    }
                }
            }
        }

        cleanupDevices();
        return false;
    }

    cleanupDevices();

    IMFSourceReader* reader = nullptr;
    hr = MFCreateSourceReaderFromMediaSource(source, nullptr, &reader);
    if (FAILED(hr)) {
        source->Release();
        return false;
    }

    setReaderOutputType(reader, opts.width, opts.height, opts.fps);

    outSource = adopt(source);
    outReader = adopt(reader);
    return true;
}

} // namespace

namespace mfcam {

struct Camera::Impl {
    com_ptr<IMFMediaSource> source;
    com_ptr<IMFSourceReader> reader;

    UINT32 width = 0;
    UINT32 height = 0;
    LONG stride = 0;
    GUID subtype = GUID_NULL;
    bool formatKnown = false;

    bool updateFormatInfo() {
        if (formatKnown) return true;
        if (!reader) return false;

        IMFMediaType* type = nullptr;
        HRESULT hr = reader->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, &type);
        if (FAILED(hr)) return false;

        UINT32 w = 0, h = 0;
        hr = MFGetAttributeSize(type, MF_MT_FRAME_SIZE, &w, &h);
        if (FAILED(hr) || w == 0 || h == 0) {
            type->Release();
            return false;
        }

        GUID st = GUID_NULL;
        hr = type->GetGUID(MF_MT_SUBTYPE, &st);
        if (FAILED(hr)) st = GUID_NULL;

        LONG s = 0;
        {
            UINT32 strideU = 0;
            hr = type->GetUINT32(MF_MT_DEFAULT_STRIDE, &strideU);
            if (SUCCEEDED(hr) && strideU != 0) s = (LONG)strideU;
        }
        if (s == 0) {
            if (st == MFVideoFormat_RGB32) {
                s = (LONG)(w * 4);
            } else if (st == MFVideoFormat_YUY2) {
                s = (LONG)(w * 2);
            } else {
                s = (LONG)w;
            }
        }

        type->Release();

        width = w;
        height = h;
        stride = s;
        subtype = st;
        formatKnown = true;
        return true;
    }
};

Camera::Camera() : impl_(new Impl()) {}

Camera::~Camera() {
    close();
    delete impl_;
    impl_ = nullptr;
}

bool Camera::openSymbolicLink(const std::string& symbolicLink, const OpenOptions& opts) {
    close();
    if (!ensureMfStarted()) return false;

    return openBySymbolicLink(symbolicLink, impl_->source, impl_->reader, opts);
}

bool Camera::readBgr(cv::Mat& outBgr) {
    outBgr.release();
    if (!impl_ || !impl_->reader) return false;

    // Try a few times to get a sample.
    for (int tries = 0; tries < 50; ++tries) {
        DWORD streamIndex = 0;
        DWORD flags = 0;
        LONGLONG ts = 0;
        IMFSample* sample = nullptr;

        HRESULT hr = impl_->reader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, &streamIndex, &flags, &ts, &sample);
        if (FAILED(hr)) return false;

        if (flags & MF_SOURCE_READERF_STREAMTICK) {
            if (sample) sample->Release();
            continue;
        }

        if (!sample) {
            Sleep(1);
            continue;
        }

        if (!impl_->updateFormatInfo()) {
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

        if (impl_->subtype == MFVideoFormat_RGB32) {
            cv::Mat bgra((int)impl_->height, (int)impl_->width, CV_8UC4, data, (size_t)impl_->stride);
            cv::cvtColor(bgra, outBgr, cv::COLOR_BGRA2BGR);
        } else if (impl_->subtype == MFVideoFormat_YUY2) {
            cv::Mat yuy2((int)impl_->height, (int)impl_->width, CV_8UC2, data, (size_t)impl_->stride);
            cv::cvtColor(yuy2, outBgr, cv::COLOR_YUV2BGR_YUY2);
        } else if (impl_->subtype == MFVideoFormat_NV12) {
            const int h = (int)impl_->height;
            const int w = (int)impl_->width;
            const size_t yStride = (size_t)impl_->stride;
            unsigned char* yPtr = data;
            unsigned char* uvPtr = data + yStride * (size_t)h;
            cv::Mat y(h, w, CV_8UC1, yPtr, yStride);
            cv::Mat uv(h / 2, w / 2, CV_8UC2, uvPtr, yStride);
            cv::cvtColorTwoPlane(y, uv, outBgr, cv::COLOR_YUV2BGR_NV12);
        } else {
            // Fallback: treat as BGRA.
            cv::Mat bgra((int)impl_->height, (int)impl_->width, CV_8UC4, data, (size_t)impl_->stride);
            cv::cvtColor(bgra, outBgr, cv::COLOR_BGRA2BGR);
        }

        buf->Unlock();
        buf->Release();
        sample->Release();

        return !outBgr.empty();
    }

    return false;
}

void Camera::close() {
    if (!impl_) return;
    impl_->reader.reset();
    impl_->source.reset();
    impl_->formatKnown = false;
}

bool loadSymbolicLinkFromCamerasYml(const fs::path& ymlPath, const std::string& camKey, std::string& outLink) {
    outLink.clear();
    try {
        YAML::Node root = YAML::LoadFile(ymlPath.string());
        if (!root[camKey]) return false;
        YAML::Node cam = root[camKey];
        if (!cam["symbolic_link"]) return false;
        outLink = cam["symbolic_link"].as<std::string>();
        return !outLink.empty();
    } catch (...) {
        return false;
    }
}

fs::path findUpwardsForFile(const fs::path& startDir, const fs::path& filename, int maxHops) {
    fs::path cur = startDir;
    for (int i = 0; i <= maxHops; ++i) {
        const fs::path candidate = cur / filename;
        if (fs::exists(candidate) && fs::is_regular_file(candidate)) {
            return candidate;
        }
        if (!cur.has_parent_path()) break;
        fs::path parent = cur.parent_path();
        if (parent == cur) break;
        cur = parent;
    }
    return {};
}

} // namespace mfcam
