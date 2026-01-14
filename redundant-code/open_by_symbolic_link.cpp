// open_by_symbolic_link.cpp
// Build: cl /EHsc open_by_symbolic_link.cpp mfplat.lib mf.lib mfreadwrite.lib mfuuid.lib
#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <algorithm>
#include <iostream>
#include <string>
#pragma comment(lib,"mfplat.lib")
#pragma comment(lib,"mf.lib")
#pragma comment(lib,"mfreadwrite.lib")
#pragma comment(lib,"mfuuid.lib")
#pragma comment(lib,"ole32.lib")

static std::wstring u8w(const char* s){ int n=MultiByteToWideChar(CP_UTF8,0,s,-1,0,0); std::wstring w(n>0?n-1:0,L'\0'); if(n>1) MultiByteToWideChar(CP_UTF8,0,s,-1,&w[0],n); return w; }
static std::wstring canon(std::wstring w){
  if(w.rfind(L"\\\\\\\\?\\",0)==0) w.erase(0,2);
  std::transform(w.begin(),w.end(),w.begin(),[](wchar_t c){ return (c>=L'A'&&c<=L'Z')?wchar_t(c+32):c; });
  if(w.rfind(L"\\\\?\\",0)==0) for(size_t p=4;(p=w.find(L"\\\\",p))!=std::wstring::npos;) w.replace(p,2,L"\\");
  return w;
}

int main(int argc,char** argv){
  if(argc<2){ std::cout<<"Usage: open_by_symbolic_link.exe \"\\\\?\\usb#...\\global\"\n"; return 1; }
  std::wstring target=canon(u8w(argv[1]));
  CoInitializeEx(nullptr,COINIT_MULTITHREADED); MFStartup(MF_VERSION);

  IMFAttributes* a=nullptr; MFCreateAttributes(&a,1);
  a->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
  IMFActivate** dev=nullptr; UINT32 n=0; MFEnumDeviceSources(a,&dev,&n); a->Release();

  IMFMediaSource* src=nullptr;
  for(UINT32 i=0;i<n;i++){ wchar_t* s=nullptr; UINT32 l=0;
    if(SUCCEEDED(dev[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK,&s,&l))){
      if(s && canon(s)==target) dev[i]->ActivateObject(IID_PPV_ARGS(&src));
      CoTaskMemFree(s);
    } dev[i]->Release(); if(src) break;
  } CoTaskMemFree(dev);
  if(!src){ std::cerr<<"Not found\n"; return 2; }

  IMFSourceReader* r=nullptr; MFCreateSourceReaderFromMediaSource(src,nullptr,&r); src->Release();
  IMFMediaType* t=nullptr; MFCreateMediaType(&t);
  t->SetGUID(MF_MT_MAJOR_TYPE,MFMediaType_Video); t->SetGUID(MF_MT_SUBTYPE,MFVideoFormat_RGB32);
  r->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM,nullptr,t); t->Release();

  for(int i=0;i<50;i++){ IMFSample* samp=nullptr; DWORD si=0,fl=0; LONGLONG ts=0;
    HRESULT hr=r->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM,0,&si,&fl,&ts,&samp);
    if(SUCCEEDED(hr) && samp){ std::cout<<"Opened OK\n"; samp->Release(); break; }
    if(samp) samp->Release(); Sleep(10);
  }
  r->Release(); MFShutdown(); CoUninitialize(); return 0;
}