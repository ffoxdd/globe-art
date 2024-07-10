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

# Main Executable

file(GLOB_RECURSE GLOBE_HEADERS src/globe/*.hpp)
set(GLOBE_SOURCES src/globe/noise_generator/anl_implementation.cpp)

add_executable(generate_globe src/generate_globe.cpp ${GLOBE_HEADERS} ${GLOBE_SOURCES})
target_include_directories(generate_globe PRIVATE ${QT5_INCLUDE_DIRS})
target_link_libraries(generate_globe PRIVATE CGAL::CGAL CGAL::CGAL_Qt5 anl)
target_compile_definitions(generate_globe PRIVATE CGAL_USE_BASIC_VIEWER QT_NO_KEYWORDS)

# Tests

pkg_search_module(GTEST REQUIRED gtest_main)
pkg_search_module(GMOCK REQUIRED gmock)
file(GLOB_RECURSE TEST_SOURCES src/globe/*_test.cpp)

add_executable(test_project)
target_sources(test_project PRIVATE ${GLOBE_HEADERS} ${GLOBE_SOURCES} ${TEST_SOURCES})
target_link_libraries(test_project PRIVATE CGAL::CGAL CGAL::CGAL_Qt5 anl ${GTEST_LDFLAGS} ${GMOCK_LDFLAGS} )
target_compile_options(test_project PRIVATE ${GTEST_CFLAGS} ${GMOCK_CFLAGS} )

enable_testing()
add_test(first_and_only_test testapp)
