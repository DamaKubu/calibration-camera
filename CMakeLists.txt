cmake_minimum_required(VERSION 3.20)

project(camera_calibration_cuda LANGUAGES CXX)

# =============================
# Compiler
# =============================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC sanity
if (MSVC)
    add_compile_options(/EHsc)
endif()

# =============================
# OpenCV (custom build)
# =============================
set(OpenCV_DIR "C:/opencv_install")
find_package(OpenCV REQUIRED)

# =============================
# Executables
# =============================
add_executable(capture
    capture/main.cpp
)

add_executable(intrinsic
    intrinsic/main.cpp
)

add_executable(extrinsic
    extrinsic/main.cpp
)

# =============================
# Optional: camera enumerator
# =============================
option(BUILD_CAMERA_TRUE_ID "Build camera_true_id device enumerator" OFF)
if (BUILD_CAMERA_TRUE_ID)
    add_executable(camera_true_id
        camera_true_id.cpp
    )
endif()

option(BUILD_DUAL_CAMERA_PREVIEW_BY_ID "Build dual_camera_preview_by_id (opens 2 cameras by symbolic_link)" OFF)
if (BUILD_DUAL_CAMERA_PREVIEW_BY_ID)
    add_executable(dual_camera_preview_by_id
        dual_camera_preview_by_id.cpp
    )
endif()

option(BUILD_STEREO_CAPTURE_PAIRS_BY_ID "Build stereo_capture_pairs_by_id (save paired images from 2 cameras)" OFF)
if (BUILD_STEREO_CAPTURE_PAIRS_BY_ID)
    add_executable(stereo_capture_pairs_by_id
        stereo_capture_pairs_by_id.cpp
    )
endif()

option(BUILD_STEREO_EXTRINSIC_FROM_PAIRS "Build stereo_extrinsic_from_pairs (cam1<->cam2)" OFF)
if (BUILD_STEREO_EXTRINSIC_FROM_PAIRS)
    add_executable(stereo_extrinsic_from_pairs
        stereo_extrinsic_from_pairs.cpp
    )
endif()

# =============================
# Link OpenCV
# =============================
foreach(target capture intrinsic extrinsic)
    target_include_directories(${target} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${target} PRIVATE ${OpenCV_LIBS})
endforeach()

if (BUILD_DUAL_CAMERA_PREVIEW_BY_ID)
    target_include_directories(dual_camera_preview_by_id PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(dual_camera_preview_by_id PRIVATE ${OpenCV_LIBS})
endif()

if (BUILD_STEREO_CAPTURE_PAIRS_BY_ID)
    target_include_directories(stereo_capture_pairs_by_id PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(stereo_capture_pairs_by_id PRIVATE ${OpenCV_LIBS})
endif()

if (BUILD_STEREO_EXTRINSIC_FROM_PAIRS)
    target_include_directories(stereo_extrinsic_from_pairs PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(stereo_extrinsic_from_pairs PRIVATE ${OpenCV_LIBS})
endif()
