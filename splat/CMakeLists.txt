# linesplat: line-camera (pushbroom) hyperspectral Gaussian splatting.
#
# Builds a CPU-only version anywhere, and adds the CUDA rasterizer when a CUDA
# compiler is found. The device code has to stay buildable on the Jetson Nano
# (JetPack 4.6: CUDA 10.2, gcc 7, sm_53), so:
#   - host code is C++17, but device code is C++14 on CUDA 10.x (its nvcc has
#     no -std=c++17) and the shared headers stick to C++14;
#   - kernels use Thrust (bundled with every toolkit) rather than CUB, which
#     only ships as its own headers from CUDA 11 on;
#   - no double-precision atomics (sm_60+) and no std::filesystem (gcc 7).
cmake_minimum_required(VERSION 3.18)
project(linesplat VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(LINESPLAT_CUDA "Build the CUDA rasterizer if a CUDA compiler is found" ON)
option(LINESPLAT_TESTS "Build the unit tests" ON)

set(LINESPLAT_HAVE_CUDA OFF)
if(LINESPLAT_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES AND NOT DEFINED ENV{CUDAARCHS})
      # "native" picks the GPU in this machine (CMake 3.24+). On a machine
      # without a GPU, or with an older CMake, pass the architecture yourself,
      # e.g. -DCMAKE_CUDA_ARCHITECTURES=53 for the Jetson Nano.
      if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
        set(CMAKE_CUDA_ARCHITECTURES native)
      else()
        set(CMAKE_CUDA_ARCHITECTURES 53)
      endif()
    endif()
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
    if(CMAKE_CUDA_COMPILER_VERSION VERSION_LESS 11.0)
      set(_cuda_std 14)
    else()
      set(_cuda_std 17)
    endif()
    set(LINESPLAT_CUDA_STANDARD ${_cuda_std} CACHE STRING
        "C++ standard for device code (14 reproduces the Jetson's CUDA 10.2)")
    set(CMAKE_CUDA_STANDARD ${LINESPLAT_CUDA_STANDARD})
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    set(LINESPLAT_HAVE_CUDA ON)
    message(STATUS "linesplat: CUDA ${CMAKE_CUDA_COMPILER_VERSION}, "
                   "architectures ${CMAKE_CUDA_ARCHITECTURES}, C++${CMAKE_CUDA_STANDARD} device code")
  else()
    message(STATUS "linesplat: no CUDA compiler found, building the CPU renderer only")
  endif()
endif()

# JSON for the dataset and scene metadata. Use the system package if there is
# one (apt: nlohmann-json3-dev, vcpkg: nlohmann-json), else download it.
find_package(nlohmann_json 3.2 QUIET)
if(NOT nlohmann_json_FOUND)
  include(FetchContent)
  FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
  FetchContent_MakeAvailable(nlohmann_json)
endif()

find_package(OpenMP)

if(MSVC)
  set(_warn /W3 /wd4996)
else()
  set(_warn -Wall -Wextra)
endif()

add_library(linesplat_core STATIC
  src/dataset.cpp
  src/npy.cpp
  src/png.cpp
  src/preview.cpp
  src/render_cpu.cpp
  src/rng.cpp
  src/scan_model.cpp
  src/scene.cpp
  src/spectra.cpp
  src/synthetic.cpp
  src/util.cpp)
target_include_directories(linesplat_core PUBLIC include)
target_link_libraries(linesplat_core PUBLIC nlohmann_json::nlohmann_json)
target_compile_options(linesplat_core PRIVATE ${_warn})
if(OpenMP_CXX_FOUND)
  target_link_libraries(linesplat_core PUBLIC OpenMP::OpenMP_CXX)
endif()

# Everything that may use the GPU links this target. With CUDA it carries the
# rasterizer and defines LINESPLAT_WITH_CUDA; without it, it's just the core.
add_library(linesplat INTERFACE)
target_link_libraries(linesplat INTERFACE linesplat_core)

if(LINESPLAT_HAVE_CUDA)
  add_library(linesplat_cuda STATIC src/cuda/rasterizer.cu)
  target_link_libraries(linesplat_cuda PUBLIC linesplat_core CUDA::cudart_static)
  target_compile_options(linesplat_cuda PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:-lineinfo>
    $<$<AND:$<COMPILE_LANGUAGE:CUDA>,$<NOT:$<CXX_COMPILER_ID:MSVC>>>:-Xcompiler=-Wall>)
  # Silence CUDA 12's "C++14 is deprecated in Thrust" notes when checking the
  # Jetson's dialect on a newer toolkit.
  target_compile_definitions(linesplat_cuda PRIVATE
    THRUST_IGNORE_DEPRECATED_CPP_DIALECT CUB_IGNORE_DEPRECATED_CPP_DIALECT
    CCCL_IGNORE_DEPRECATED_CPP_DIALECT)
  target_compile_definitions(linesplat_cuda PUBLIC LINESPLAT_WITH_CUDA=1)
  target_link_libraries(linesplat INTERFACE linesplat_cuda)
endif()

foreach(tool splat_synth splat_render)
  add_executable(${tool} tools/${tool}.cpp)
  target_link_libraries(${tool} PRIVATE linesplat)
  target_compile_options(${tool} PRIVATE ${_warn})
endforeach()

if(LINESPLAT_TESTS)
  enable_testing()
  add_executable(splat_tests
    tests/test_main.cpp
    tests/test_math.cpp
    tests/test_projection.cpp
    tests/test_scan_model.cpp
    tests/test_render_cpu.cpp
    tests/test_io.cpp
    tests/test_synthetic.cpp
    tests/test_cuda.cpp)
  target_link_libraries(splat_tests PRIVATE linesplat)
  target_compile_options(splat_tests PRIVATE ${_warn})
  # Scratch files go in the build tree, wherever the tests are run from.
  target_compile_definitions(splat_tests PRIVATE LINESPLAT_TEST_TMP="${CMAKE_CURRENT_BINARY_DIR}/splat_test_tmp")
  add_test(NAME splat_tests COMMAND splat_tests)
endif()
