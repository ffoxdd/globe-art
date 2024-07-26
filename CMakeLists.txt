cmake_minimum_required(VERSION 3.1...3.23)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(GlobeArt)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE true) # remove for release

# Libraries

find_package(PkgConfig REQUIRED)

set(Qt5_DIR "/opt/homebrew/opt/qt@5/lib/cmake/Qt5")
find_package(CGAL QUIET COMPONENTS Qt5)
include_directories(BEFORE SYSTEM /opt/homebrew/Cellar/qt@5/5.15.13_1/include)
add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)

find_package(Boost REQUIRED)
find_package(anl REQUIRED)
find_package(TBB REQUIRED)

# Experiments

add_executable(test_noise_sphere src/experiments/test_noise_sphere.cpp src/globe/noise_generator/anl_implementation.cpp)
target_include_directories(test_noise_sphere PRIVATE ${QT5_INCLUDE_DIRS})
target_link_libraries(test_noise_sphere PRIVATE CGAL::CGAL CGAL::CGAL_Qt5 anl TBB::tbb)
target_compile_definitions(test_noise_sphere PRIVATE CGAL_USE_BASIC_VIEWER QT_NO_KEYWORDS)

# Main Executable

file(GLOB_RECURSE GLOBE_HEADERS src/globe/*.hpp)
set(GLOBE_SOURCES src/globe/noise_generator/anl_implementation.cpp)

add_executable(generate_globe src/generate_globe.cpp ${GLOBE_HEADERS} ${GLOBE_SOURCES})
target_include_directories(generate_globe PRIVATE ${QT5_INCLUDE_DIRS})
target_link_libraries(generate_globe PRIVATE CGAL::CGAL CGAL::CGAL_Qt5 anl TBB::tbb)
target_compile_definitions(generate_globe PRIVATE CGAL_USE_BASIC_VIEWER QT_NO_KEYWORDS)

# Tests

pkg_search_module(GTEST REQUIRED gtest_main)
pkg_search_module(GMOCK REQUIRED gmock)

file(GLOB_RECURSE TEST_SOURCES src/globe/*_test.cpp)

foreach (TEST_SRC ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SRC} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SRC} ${GLOBE_SOURCES})
    target_link_libraries(${TEST_NAME} PRIVATE CGAL::CGAL CGAL::CGAL_Qt5 anl TBB::tbb ${GTEST_LDFLAGS} ${GMOCK_LDFLAGS})
    target_compile_options(${TEST_NAME} PRIVATE ${GTEST_CFLAGS} ${GMOCK_CFLAGS})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach ()

enable_testing()
