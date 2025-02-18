cmake_minimum_required(VERSION 3.1...3.23)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(GlobeArt)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE true) # remove for release

set(CMAKE_DISABLE_FIND_PACKAGE_WrapVulkanHeaders ON)
cmake_policy(SET CMP0167 OLD)  # "Policy CMP0167 is not set: The FindBoost module is removed."

# Libraries

find_package(PkgConfig REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui OpenGLWidgets Svg)
find_package(CGAL REQUIRED COMPONENTS Qt6)
add_definitions(-DCGAL_USE_BASIC_VIEWER)

find_package(Boost REQUIRED CONFIG)
find_package(anl REQUIRED)
find_package(TBB REQUIRED CONFIG)

# Main Executable

file(GLOB_RECURSE GLOBE_HEADERS src/globe/*.hpp)
set(GLOBE_SOURCES src/globe/noise_generator/anl_implementation.cpp)

add_executable(generate_globe src/generate_globe.cpp ${GLOBE_HEADERS} ${GLOBE_SOURCES})
target_include_directories(generate_globe PRIVATE Qt6::Widgets ${CGAL_INCLUDE_DIRS})
target_link_libraries(generate_globe PRIVATE CGAL::CGAL CGAL::CGAL_Qt6 anl TBB::tbb)
target_compile_definitions(generate_globe PRIVATE CGAL_USE_BASIC_VIEWER QT_NO_KEYWORDS)

# Tests

pkg_search_module(GTEST REQUIRED gtest_main)
pkg_search_module(GMOCK REQUIRED gmock)

file(GLOB_RECURSE TEST_SOURCES src/globe/*_test.cpp)

foreach (TEST_SRC ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SRC} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SRC} ${GLOBE_SOURCES})
    target_link_libraries(${TEST_NAME} PRIVATE CGAL::CGAL CGAL::CGAL_Qt6 anl TBB::tbb ${GTEST_LDFLAGS} ${GMOCK_LDFLAGS})
    target_compile_options(${TEST_NAME} PRIVATE ${GTEST_CFLAGS} ${GMOCK_CFLAGS})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach ()

enable_testing()
