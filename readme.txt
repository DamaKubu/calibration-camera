CAMERA CALIBRATION (OpenCV + MSVC + CMake) — Windows

Date: 2026-01-12

=====================================================
1) What this project is
=====================================================
This repo builds small calibration tools (C++17) using OpenCV:

- capture   : saves calibration images from a camera
- intrinsic : computes camera intrinsics from a folder of chessboard images
- extrinsic : estimates a single pose (rvec/tvec) using intrinsics + one image

Optional:
- camera_true_id : lists Windows camera device IDs (Media Foundation) so you can
                   bind calibration to the real physical device.

Folder layout:
- CMakeLists.txt          : the ONLY CMake file you run
- capture/main.cpp        : capture tool
- intrinsic/main.cpp      : intrinsic tool (offline calibrator + outlier rejection)
- extrinsic/main.cpp      : extrinsic tool
- camera_true_id.cpp      : optional device enumerator
- data/                   : input images + output YAML files

=====================================================
2) Requirements (hard)
=====================================================
- Windows 10/11 x64
- Visual Studio 2022 (MSVC x64)
- CMake
- OpenCV already installed at:
    C:\opencv_install
- OpenCV DLLs are already on PATH:
    C:\opencv_install\x64\vc17\bin

Notes:
- CUDA support is assumed to be compiled into your OpenCV build.
- No Python, no vcpkg, no Conan, no FetchContent.

=====================================================
3) Build (recommended)
=====================================================
Open an "x64 Native Tools Command Prompt for VS 2022".

From the project root (the folder that contains CMakeLists.txt):

  cd C:\Users\IT Logika\Documents\CALIBRATION

  cmake -S . -B build -G "Visual Studio 17 2022"
  cmake --build build --config Release



Outputs:
- build\Release\capture.exe
- build\Release\intrinsic.exe
- build\Release\extrinsic.exe

=====================================================
4) Runtime DLL rule (important)
=====================================================
This project does NOT copy DLLs.

Make sure OpenCV DLLs are on PATH when running:
  C:\opencv_install\x64\vc17\bin

If needed (run once, then restart terminal/VS Code):
  setx PATH "C:\opencv_install\x64\vc17\bin;%PATH%"

=====================================================
5) Data folders
=====================================================
Your data folders look like:

  data\calib_cam1\calib_0000.png ...
  data\calib_cam2\calib_0000.png ...

The new intrinsic tool reads any *.png in the directory (default), so both
"img_*.png" and "calib_*.png" are fine.

=====================================================
6) Using the tools
=====================================================
Tip: You can run from anywhere; intrinsic auto-finds the project root "data/".

(A) Capture images
------------------
  build\Release\capture.exe --camera 0 --camera-id calib_cam1 --count 60

Saves images to:
  data\<camera-id>\img_0000.png ...

(B) Intrinsic calibration
-------------------------
Typical (uses data/<camera-id>/):
  build\Release\intrinsic.exe --camera-id calib_cam1 --pattern 8x5 --square-mm 65 --model standard

Or explicitly:
  build\Release\intrinsic.exe --images-dir data\calib_cam1 --output data\calib_cam1\intrinsic.yml

Outputs:
  data\calib_cam1\intrinsic.yml

The YAML includes:
- camera_matrix
- distortion_coefficients
- reprojection stats

(C) Extrinsic calibration
-------------------------
  build\Release\extrinsic.exe --camera-id calib_cam1 --pattern 8x5 --square-mm 65

Reads:
  data\calib_cam1\intrinsic.yml
Writes:
  data\calib_cam1\extrinsic.yml

=====================================================
7) Optional: identify the real camera (stable device ID)
=====================================================
Why:
- OpenCV camera indexes (0,1,2,...) can change between reboots/unplugging.
- Windows Media Foundation exposes a "symbolic_link" string that is much more
  stable for identifying a physical device.

Build with the optional target:
  cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DBUILD_CAMERA_TRUE_ID=ON
  cmake --build build --config Release

Run:
  build\Release\camera_true_id.exe

Store the printed symbolic_link inside your calibration YAML so you always know
which camera the calibration belongs to.

  =====================================================
  8) Stereo: capture PAIRS + stereo extrinsics
  =====================================================

  Why your stereo calibration got 0 detections:
  - The stereo solver needs BOTH images in a pair to contain a detectable chessboard.
  - If you saved random frames (no board / blurry / wrong pattern size), it will report 0 valid pairs.

  (A) Build optional stereo tools
  ------------------------------
    cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
        -DBUILD_STEREO_CAPTURE_PAIRS_BY_ID=ON ^
        -DBUILD_STEREO_EXTRINSIC_FROM_PAIRS=ON
    cmake --build build --config Release

=====================================================
9) Single camera: capture + extrinsic (by cameras.yml)
=====================================================

Build the optional tool:
  cmake -S . -B build -G "Visual Studio 17 2022" -DBUILD_SINGLE_CAMERA_CAPTURE_AND_EXTRINSIC_BY_YML=ON
  cmake --build build --config Release

Run (example for cam1):
  build\Release\single_camera_capture_and_extrinsic_by_yml.exe --cameras-yml cameras.yml --cam cam1 \
      --intrinsic data\calib_cam1\intrinsic.yml --output data\calib_cam1\extrinsic_live.yml \
      --pattern 8x5 --square-mm 65 --save-dir data\calib_cam1 --count 30

Controls:
- Space saves one image
- E solves extrinsic (solvePnP) on the current frame and writes the YAML

  (B) Capture paired images (by symbolic_link)
  -------------------------------------------
  Important: quoting depends on your shell.

  - In cmd.exe (your prompt looks like C:\path>):
    - Single quotes DO NOT quote. The '&' in the symbolic link will break the command.
    - Use double quotes "...".

  - In PowerShell (prompt looks like PS C:\path>):
    - Single quotes '...' are OK.

  Recommended: use AUTO mode so it only saves when BOTH cameras detect the board:

    build\Release\stereo_capture_pairs_by_id.exe --cam1 "<symbolic_link_1>" --cam2 "<symbolic_link_2>" \
        --out-dir data\pairs_cam1_cam2 --count 30 --pattern 8x5 --auto --stable 5 --min-ms 350

  What you should see:
  - Each window shows "FOUND" / "NO" and draws corners when detected.
  - It will save automatically when both are FOUND for N frames.

  (C) Solve stereo extrinsics
  --------------------------
    build\Release\stereo_extrinsic_from_pairs.exe --pairs-dir data\pairs_cam1_cam2 \
        --intr1 data\calib_cam1\intrinsic.yml --intr2 data\calib_cam2\intrinsic.yml \
        --pattern 8x5 --square-mm 65 --output data\pairs_cam1_cam2\stereo_extrinsic.yml

  Notes:
  - --pattern is INNER corners (so a 9x6 square board is --pattern 8x5).
  - If it still says 0 detections, try swapping: --pattern 5x8 and recapture.




C:\Users\IT Logika\Documents\CALIBRATION\build\Release>.\stereo_capture_pairs_by_id.exe --cam1 '\\?\usb#vid_1bcf&pid_2286&mi_00#7&12826ac5&0&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global' --cam2 '\\?\usb#vid_1bcf&pid_2286&mi_00#6&27cb31a1&0&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global' --out-dir ..\..\data\pairs_cam1_cam2 --count 30
ERROR: --cam1 and --cam2 are required
stereo_capture_pairs_by_id

Usage:
  stereo_capture_pairs_by_id.exe --cam1 <symbolic_link> --cam2 <symbolic_link> --out-dir data/pairs_cam1_cam2 [--count 30]

Controls:
  Space : save one synchronized pair
  Esc   : quit
'pid_2286' is not recognized as an internal or external command,
operable program or batch file.
'mi_00#7' is not recognized as an internal or external command,
operable program or batch file.
'12826ac5' is not recognized as an internal or external command,
operable program or batch file.
'0' is not recognized as an internal or external command,
operable program or batch file.
The system cannot find the path specified.
'pid_2286' is not recognized as an internal or external command,
operable program or batch file.
'mi_00#6' is not recognized as an internal or external command,
operable program or batch file.
'27cb31a1' is not recognized as an internal or external command,
operable program or batch file.
'0' is not recognized as an internal or external command,
operable program or batch file.
The system cannot find the path specified.

C:\Users\IT Logika\Documents\CALIBRATION\build\Release>.\stereo_capture_pairs_by_id.exe --cam1 "\\?\usb#vid_1bcf&pid_2286&mi_00#7&12826ac5&0&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global" --cam2 "\\?\usb#vid_1bcf&pid_2286&mi_00#6&27cb31a1&0&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global" --out-dir ..\..\data\pairs_cam1_cam2 --count 30
Saving pairs to: C:\Users\IT Logika\Documents\CALIBRATION\data\pairs_cam1_cam2
Press Space to save pair, Esc to quit.
Saved pair 1/30 (cam1_0000.png, cam2_0000.png)
Saved pair 2/30 (cam1_0001.png, cam2_0001.png)
Saved pair 3/30 (cam1_0002.png, cam2_0002.png)
Saved pair 4/30 (cam1_0003.png, cam2_0003.png)
Saved pair 5/30 (cam1_0004.png, cam2_0004.png)
Saved pair 6/30 (cam1_0005.png, cam2_0005.png)
Saved pair 7/30 (cam1_0006.png, cam2_0006.png)
Saved pair 8/30 (cam1_0007.png, cam2_0007.png)
Saved pair 9/30 (cam1_0008.png, cam2_0008.png)
Saved pair 10/30 (cam1_0009.png, cam2_0009.png)
Saved pair 11/30 (cam1_0010.png, cam2_0010.png)
Saved pair 12/30 (cam1_0011.png, cam2_0011.png)
Saved pair 13/30 (cam1_0012.png, cam2_0012.png)
Saved pair 14/30 (cam1_0013.png, cam2_0013.png)
Saved pair 15/30 (cam1_0014.png, cam2_0014.png)
Saved pair 16/30 (cam1_0015.png, cam2_0015.png)
Saved pair 17/30 (cam1_0016.png, cam2_0016.png)
Saved pair 18/30 (cam1_0017.png, cam2_0017.png)
Saved pair 19/30 (cam1_0018.png, cam2_0018.png)
Saved pair 20/30 (cam1_0019.png, cam2_0019.png)
Saved pair 21/30 (cam1_0020.png, cam2_0020.png)
Saved pair 22/30 (cam1_0021.png, cam2_0021.png)
Saved pair 23/30 (cam1_0022.png, cam2_0022.png)
Saved pair 24/30 (cam1_0023.png, cam2_0023.png)
Saved pair 25/30 (cam1_0024.png, cam2_0024.png)
Saved pair 26/30 (cam1_0025.png, cam2_0025.png)
Saved pair 27/30 (cam1_0026.png, cam2_0026.png)
Saved pair 28/30 (cam1_0027.png, cam2_0027.png)
Saved pair 29/30 (cam1_0028.png, cam2_0028.png)
Saved pair 30/30 (cam1_0029.png, cam2_0029.png)

C:\Users\IT Logika\Documents\CALIBRATION\build\Release>ls
camera_true_id.exe             extrinsic.exe
capture.exe                    intrinsic.exe
data                           stereo_capture_pairs_by_id.exe
dual_camera_preview_by_id.exe  stereo_extrinsic_from_pairs.exe

C:\Users\IT Logika\Documents\CALIBRATION\build\Release>.\stereo_extrinsic_from_pairs.exe --pairs-dir ..\..\data\pairs_cam1_cam2 --intr1 ..\..\data\calib_cam1\intrinsic.yml --intr2 ..\..\data\calib_cam2\intrinsic.yml --pattern 8x5 --square-mm 65 --output ..\..\data\pairs_cam1_cam2\stereo_extrinsic.yml
ERROR: Not enough valid stereo detections: 0 (need >= 8)





