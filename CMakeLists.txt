cmake_minimum_required(VERSION 3.26)
project(GlobeArt CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE true)
set(CMAKE_CXX_SCAN_FOR_MODULES ON)

# Ensure the compiler is GCC or Clang and check their versions
if (NOT ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "11.0") OR
(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "13.0")))
    message(FATAL_ERROR "GCC 11 or higher or Clang 13 or higher is required for C++20 modules.")
endif()

# Set global compile options for module support based on the compiler
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-fmodules-ts)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    add_compile_options(-fmodules -fcxx-modules)
endif()

# Libraries
find_package(PkgConfig REQUIRED)

set(Qt5_DIR "/opt/homebrew/opt/qt@5/lib/cmake/Qt5")
find_package(CGAL QUIET COMPONENTS Qt5)
add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
include_directories(BEFORE SYSTEM /opt/homebrew/Cellar/qt@5/5.15.13_1/include)

find_package(Boost REQUIRED)
find_package(anl REQUIRED)

# Main Application
set(
        GLOBE_HEADERS
        src/globe/globe_viewer.hpp
        src/globe/globe_generator.hpp
        src/globe/point_generator/point_generator.hpp
        src/globe/point_generator/mock_point_generator.hpp
        src/globe/point_generator/random_sphere_point_generator.hpp
        src/globe/sphere_mesh_generator/sphere_mesh_generator.hpp
        src/globe/noise_generator/point_iterator.hpp
        src/globe/noise_generator/range.hpp
        src/globe/noise_generator/anl_noise_generator.hpp
        src/globe/noise_generator/noise_generator.hpp
        src/globe/geometry/helpers.hpp
        src/globe/types.hpp
)

set(
        GLOBE_SOURCES
        src/generate_globe.cpp
        src/globe/point_generator/random_sphere_point_generator.cpp
        src/globe/points_collection/points_collection.cpp
        src/globe/globe_viewer.cpp
        src/globe/noise_generator/range.cpp
        src/globe/noise_generator/anl_implementation.cpp
        src/globe/noise_generator/point_iterator.cpp
)

add_library(math src/globe/math.cpp)
target_sources(math PUBLIC FILE_SET CXX_MODULES FILES src/globe/math.ixx)

# Main application
add_executable(generate_globe ${GLOBE_HEADERS} ${GLOBE_SOURCES})
target_link_libraries(generate_globe PRIVATE CGAL::CGAL CGAL::CGAL_Qt5 anl math)
target_include_directories(generate_globe PRIVATE ${QT5_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS})
target_compile_definitions(generate_globe PRIVATE CGAL_USE_BASIC_VIEWER QT_NO_KEYWORDS)


# Tests
pkg_search_module(GTEST REQUIRED gtest_main)
pkg_search_module(GMOCK REQUIRED gmock)
file(GLOB_RECURSE TEST_SOURCES src/globe/*_test.cpp)

add_executable(test_project ${GLOBE_HEADERS} ${GLOBE_SOURCES} ${TEST_SOURCES})
target_include_directories(test_project PRIVATE ${QT5_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS})
target_link_libraries(test_project PRIVATE CGAL::CGAL CGAL::CGAL_Qt5 anl ${GTEST_LDFLAGS} ${GMOCK_LDFLAGS})
target_compile_options(test_project PRIVATE ${GTEST_CFLAGS} ${GMOCK_CFLAGS})

enable_testing()
add_test(first_and_only_test test_project)