mf_camera mini-library

Files:
- mf_camera.h
- mf_camera.cpp

Purpose:
- Open a Windows camera by Media Foundation symbolic_link (stable device ID)
- Read frames into OpenCV cv::Mat (BGR)

Typical use:
  mfcam::Camera cam;
  mfcam::OpenOptions opts{1080,1920,30};
  cam.openSymbolicLink(symbolicLink, opts);
  cv::Mat frame;
  cam.readBgr(frame);

Link libs (MSVC):
- mfplat.lib mf.lib mfreadwrite.lib mfuuid.lib ole32.lib

This is intended to be copied into another project as-is.
