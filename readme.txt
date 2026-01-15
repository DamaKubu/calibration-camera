CAMERA CALIBRATION (OpenCV + MSVC + CMake) — Windows

Date: 2026-01-12

=====================================================
1) What this project is
=====================================================
This repo builds small calibration tools (C++17) using OpenCV:

- capture   : saves calibration images from a camera
- capture_multiple : saves synchronized shots for multi-camera extrinsic (2..6 cams)
- intrinsic : computes camera intrinsics from a folder of chessboard images
- extrinsic : estimates a single pose (rvec/tvec) using intrinsics + one image

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
- YAML parsing uses yaml-cpp.
  - If yaml-cpp is installed, CMake will use it.
  - Otherwise CMake will FetchContent yaml-cpp (needs Git + internet at configure time).

=====================================================
3) Build (recommended)
=====================================================
Open an "x64 Native Tools Command Prompt for VS 2022".

From the project root (the folder that contains CMakeLists.txt):

  cd C:\Users\IT Logika\Documents\CALIBRATION

  cmake -S . -B build -G "Visual Studio 17 2022" -A x64
  cmake --build build --config Release



Outputs:
- build\Release\capture.exe
- build\Release\capture_multiple.exe
- build\Release\intrinsic.exe
- build\Release\extrinsic.exe
- build\Release\camera_true_id.exe

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

Run:
  build\Release\camera_true_id.exe

Store the printed symbolic_link inside your calibration YAML so you always know
which camera the calibration belongs to.

IMPORTANT cameras.yml formatting:
- Use single quotes for symbolic_link values, like:
    symbolic_link: '\\?\usb#vid_....\\global'
  This keeps it valid YAML ("\\u" inside double quotes is treated as an escape).

=====================================================
8) Multi-camera capture (2..6 cameras): capture_multiple
=====================================================

Goal:
- Capture synchronized "shots" for multi-camera extrinsic calibration.
- It only shows GOOD when ALL cameras detect the chessboard and the board is stable.

Run (example 2 cams):
  build\Release\capture_multiple.exe --cams cam1,cam2 --count 60 --auto

Run (example 4 cams):
  build\Release\capture_multiple.exe --cams cam1,cam2,cam3,cam4 --count 80 --auto

Output:
- data\extrinsic_multi\session_YYYYMMDD_HHMMSS\
  - shot_0000_cam1.png, shot_0000_cam2.png, ...
  - shot_0000.yml (per-shot manifest)
  - session.yml (session metadata)

Controls:
- Space : save one shot (all cams)
- A     : toggle auto mode
- Esc   : quit

Quality tuning (for high precision):
- --max-motion-px 0.25   (lower = stricter)
- --stable-frames 6      (higher = stricter)
- --min-border-px 20     (increase to avoid edge distortion)
- --min-board-px 20      (increase to force closer board)

CUDA note:
- You can try --cuda-preprocess to run grayscale conversion on GPU if your OpenCV build has cudaimgproc.

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



Simplified live_motion (chessboard-only)
----------------------------------------

flowchart TD
  A[Start live_motion.exe] --> B[Load intrinsics for cam1/cam2]
  B --> C[Load stereo extrinsics R,T]
  C --> D[Open cam1 & cam2 streams]
  D --> E[Detect chessboard corners (9x6)]
  E --> F[Get 2D center in cam1 + cam2]
  F --> G[Undistort to normalized rays]
  G --> H[Triangulate with R,T]
  H --> I[Show XYZ + save CSV]

Example:
  build\Release\live_motion.exe --cam1 cam1 --cam2 cam2 --board --pattern 9x6 --square-mm 23.6 \
      --extrinsics reports\stereo_live.yml --baseline-mm 980 --max-fps 15


3-camera calibration workflow (intrinsics + extrinsics)
------------------------------------------------------

flowchart TD
  A[Capture intrinsics per camera] --> B[Run intrinsic.exe per camera]
  B --> C[Capture multi-camera session]
  C --> D[Run extrinsic.exe on session]
  D --> E[Use extrinsics with live_motion]

Step-by-step (3 cams: cam1, cam2, cam3)
---------------------------------------

1) Capture intrinsics for each camera:
  build\Release\capture.exe --camera 0 --camera-id calib_cam1 --count 60
  build\Release\capture.exe --camera 1 --camera-id calib_cam2 --count 60
  build\Release\capture.exe --camera 2 --camera-id calib_cam3 --count 60

2) Solve intrinsics for each camera:
  build\Release\intrinsic.exe --camera-id calib_cam1 --pattern 9x6 --square-mm 23.6
  build\Release\intrinsic.exe --camera-id calib_cam2 --pattern 9x6 --square-mm 23.6
  build\Release\intrinsic.exe --camera-id calib_cam3 --pattern 9x6 --square-mm 23.6

3) Capture synchronized multi-camera session:
  build\Release\capture_multiple.exe --cams cam1,cam2,cam3 --count 60 --auto --pattern 9x6

4) Solve extrinsics for the session:
  build\Release\extrinsic.exe --session data\extrinsic_multi\session_0 --ref cam1

Notes:
- Make sure capture resolution matches the intrinsics calibration resolution.
- Use large board coverage and varied tilts for stable extrinsics.


live_motion program flow (easy to modify)
----------------------------------------

flowchart TD
  A[Parse CLI + AppConfig] --> B[Load intrinsics]
  B --> C[Load or quick-calibrate stereo extrinsics]
  C --> D[Open cameras]
  D --> E[Main loop]
  E --> F{Tracking mode}
  F -->|board| G[Detect chessboard]
  G --> K[2D points]
  K --> L[Undistort + triangulate]
  L --> M[Display + CSV]
  M --> E



  
