cmake_minimum_required(VERSION 3.16)
project(powercrust VERSION 1.3 LANGUAGES C CXX)

# ── Language standards ────────────────────────────────────────────────────────
set(CMAKE_C_STANDARD   11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Build type default ────────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ── Common compiler flags ─────────────────────────────────────────────────────
add_compile_options(-Wall -Wextra)

# -O3 in release builds
if(CMAKE_BUILD_TYPE MATCHES "Release|RelWithDebInfo")
  add_compile_options(-O3)
endif()

# ── SIMD: AVX2 + FMA on x86-64 ───────────────────────────────────────────────
include(CheckCCompilerFlag)
check_c_compiler_flag(-mavx2 HAVE_AVX2)
check_c_compiler_flag(-mfma  HAVE_FMA)
if(HAVE_AVX2 AND HAVE_FMA)
  add_compile_options(-mavx2 -mfma)
  message(STATUS "AVX2 + FMA enabled")
else()
  message(STATUS "AVX2/FMA not available; using SSE2 / scalar fallback")
endif()

# ── OpenMP ────────────────────────────────────────────────────────────────────
find_package(OpenMP)
if(OpenMP_FOUND)
  message(STATUS "OpenMP found: version ${OpenMP_C_VERSION}")
endif()

# ── zlib (optional — for GZipBase64 GIFTI support) ───────────────────────────
find_package(ZLIB)
if(ZLIB_FOUND)
  add_compile_definitions(HAVE_ZLIB)
  message(STATUS "zlib found: GZipBase64 GIFTI enabled")
else()
  message(STATUS "zlib not found: GZipBase64 GIFTI disabled")
endif()

# ── Core library (C sources shared by all three targets) ─────────────────────
set(LIB_C_SOURCES
    src/hull.c
    src/ch.c
    src/io.c
    src/crust.c
    src/power.c
    src/rand.c
    src/pointops.c
    src/fg.c
    src/math.c
    src/predicates.c
    src/heap.c
    src/label.c
)

# io_mesh.cpp is C++17
add_library(powercrust_core STATIC ${LIB_C_SOURCES} src/io_mesh.cpp)

target_include_directories(powercrust_core PUBLIC src)

if(OpenMP_FOUND)
  target_link_libraries(powercrust_core PUBLIC OpenMP::OpenMP_C OpenMP::OpenMP_CXX)
endif()
if(ZLIB_FOUND)
  target_link_libraries(powercrust_core PUBLIC ZLIB::ZLIB)
endif()
target_link_libraries(powercrust_core PUBLIC m)

# ── powercrust executable ─────────────────────────────────────────────────────
add_executable(powercrust src/hullmain.c)
target_link_libraries(powercrust PRIVATE powercrust_core)

# ── simplify executable ───────────────────────────────────────────────────────
add_executable(simplify src/powershape.C)
target_link_libraries(simplify PRIVATE m)
if(OpenMP_FOUND)
  target_link_libraries(simplify PRIVATE OpenMP::OpenMP_CXX)
endif()

# ── orient executable ─────────────────────────────────────────────────────────
add_executable(orient src/setNormals.C)
target_link_libraries(orient PRIVATE m)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS powercrust simplify orient
        RUNTIME DESTINATION bin)

# ── Summary ───────────────────────────────────────────────────────────────────
message(STATUS "")
message(STATUS "PowerCrust ${PROJECT_VERSION} configuration")
message(STATUS "  Build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "  C standard : C${CMAKE_C_STANDARD}")
message(STATUS "  C++        : C++${CMAKE_CXX_STANDARD}")
message(STATUS "  OpenMP     : ${OpenMP_FOUND}")
message(STATUS "  AVX2+FMA   : ${HAVE_AVX2}")
message(STATUS "  zlib/GZip  : ${ZLIB_FOUND}")
message(STATUS "")
