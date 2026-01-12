#include <windows.h>

#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>

#include <iomanip>
#include <iostream>
#include <regex>
#include <string>
#include <vector>
#include <memory>

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfuuid.lib")

static std::string wideToUtf8(const wchar_t* w) {
    if (!w) return {};
    int len = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    if (len <= 0) return {};
    std::unique_ptr<char[]> buf(new char[(size_t)len]);
    WideCharToMultiByte(CP_UTF8, 0, w, -1, reinterpret_cast<LPSTR>(buf.get()), len, nullptr, nullptr);
    std::string out(buf.get());
    return out;
}

static void printHresult(const char* what, HRESULT hr) {
    std::cerr << what << " failed (HRESULT=0x" << std::hex << std::setw(8) << std::setfill('0')
              << (unsigned)hr << std::dec << ")\n";
}

static void tryPrintVidPid(const std::string& s) {
    // Many symbolic links contain ...#vid_046d&pid_085e... etc.
    std::regex re("vid_([0-9a-fA-F]{4}).*pid_([0-9a-fA-F]{4})");
    std::smatch m;
    if (std::regex_search(s, m, re) && m.size() == 3) {
        std::cout << "    vid: " << m[1].str() << "\n";
        std::cout << "    pid: " << m[2].str() << "\n";
    }
}

int main() {
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

    IMFAttributes* attr = nullptr;
    hr = MFCreateAttributes(&attr, 1);
    if (FAILED(hr)) {
        printHresult("MFCreateAttributes", hr);
        MFShutdown();
        CoUninitialize();
        return 1;
    }

    hr = attr->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
    if (FAILED(hr)) {
        printHresult("SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE)", hr);
        attr->Release();
        MFShutdown();
        CoUninitialize();
        return 1;
    }

    IMFActivate** devices = nullptr;
    UINT32 count = 0;
    hr = MFEnumDeviceSources(attr, &devices, &count);
    if (FAILED(hr)) {
        printHresult("MFEnumDeviceSources", hr);
        attr->Release();
        MFShutdown();
        CoUninitialize();
        return 1;
    }

    std::cout << "Found " << count << " video capture device(s).\n\n";

    for (UINT32 i = 0; i < count; ++i) {
        wchar_t* nameW = nullptr;
        UINT32 nameLen = 0;
        wchar_t* linkW = nullptr;
        UINT32 linkLen = 0;

        // Friendly name
        if (SUCCEEDED(devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &nameW, &nameLen))) {
            std::string name = wideToUtf8(nameW);
            CoTaskMemFree(nameW);
            nameW = nullptr;

            std::cout << "[" << i << "] " << name << "\n";
        } else {
            std::cout << "[" << i << "] (unknown name)\n";
        }

        // Stable-ish device ID: symbolic link
        if (SUCCEEDED(devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &linkW, &linkLen))) {
            std::string link = wideToUtf8(linkW);
            CoTaskMemFree(linkW);
            linkW = nullptr;

            std::cout << "    symbolic_link: " << link << "\n";
            tryPrintVidPid(link);
        } else {
            std::cout << "    symbolic_link: (unavailable)\n";
        }

        std::cout << "\n";
    }

    for (UINT32 i = 0; i < count; ++i) {
        devices[i]->Release();
    }
    CoTaskMemFree(devices);
    attr->Release();

    MFShutdown();
    CoUninitialize();

    std::cout << "Tip: store symbolic_link in your intrinsic.yml/extrinsic.yml to bind calibration to a physical camera.\n";
    return 0;
}
