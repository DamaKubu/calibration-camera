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
# YAML (yaml-cpp)
# =============================
# Prefer an installed yaml-cpp (e.g. via vcpkg), otherwise fetch it.
find_package(yaml-cpp CONFIG QUIET)
if (NOT yaml-cpp_FOUND)
    include(FetchContent)
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG 0.8.0
    )
    FetchContent_MakeAvailable(yaml-cpp)
endif()

if (TARGET yaml-cpp::yaml-cpp)
    set(YAMLCPP_TARGET yaml-cpp::yaml-cpp)
elseif (TARGET yaml-cpp)
    set(YAMLCPP_TARGET yaml-cpp)
else()
    message(FATAL_ERROR "yaml-cpp target not found after configuration")
endif()

# =============================
# Shared camera module (Media Foundation)
# =============================
add_library(mf_camera STATIC
    capture/mf_camera.cpp
)
target_include_directories(mf_camera PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(mf_camera PRIVATE ${OpenCV_LIBS} ${YAMLCPP_TARGET})
if (WIN32)
    target_link_libraries(mf_camera PRIVATE mfplat mf mfreadwrite mfuuid ole32)
endif()

# =============================
# Executables (essentials)
# =============================
add_executable(capture capture/main.cpp)
add_executable(capture_multiple capture/capture_multiple/main.cpp)
add_executable(capture_intrinsic
    capture/capture_intrinsic/main.cpp
    capture/capture_intrinsic/capture_intrinsic_app.cpp
)
add_executable(intrinsic
    intrinsic/main.cpp
    intrinsic/intrinsic_app.cpp
)
add_executable(extrinsic
    extrinsic/main.cpp
    extrinsic/extrinsic_app.cpp
)
add_executable(live_motion
    live_motion/main.cpp
    live_motion/live_motion_app.cpp
)
add_executable(stereo_gpu_min tools/stereo_gpu_min.cpp)
add_executable(single_red_tracking tools/single_red_tracking.cpp)
add_executable(camera_true_id camera_true_id.cpp)
add_executable(stereo_extrinsic_from_pairs stereo_extrinsic_from_pairs.cpp)


# =============================
# Link
# =============================
foreach(target capture capture_multiple capture_intrinsic intrinsic extrinsic live_motion stereo_gpu_min single_red_tracking camera_true_id)
    target_include_directories(${target} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${target} PRIVATE ${OpenCV_LIBS})
endforeach()

target_link_libraries(capture PRIVATE mf_camera)
target_link_libraries(capture_multiple PRIVATE mf_camera)
target_link_libraries(capture_intrinsic PRIVATE mf_camera)
target_link_libraries(live_motion PRIVATE mf_camera)

if (WIN32)
    target_link_libraries(camera_true_id PRIVATE mfplat mf mfuuid ole32)
endif()

if (TARGET stereo_extrinsic_from_pairs)
    target_include_directories(stereo_extrinsic_from_pairs PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(stereo_extrinsic_from_pairs PRIVATE ${OpenCV_LIBS})
endif()
