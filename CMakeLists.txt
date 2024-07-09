cmake_minimum_required(VERSION 3.26)
project(GlobeArt CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE true)
set(CMAKE_CXX_SCAN_FOR_MODULES ON)

if (
(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "11.0") OR
(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "13.0")
)
    message(FATAL_ERROR "GCC 11 or higher or Clang 13 or higher is required for C++20 modules.")
endif ()

# Libraries

find_package(PkgConfig REQUIRED)

set(Qt5_DIR "/opt/homebrew/opt/qt@5/lib/cmake/Qt5")
find_package(CGAL QUIET COMPONENTS Qt5)
add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
include_directories(BEFORE SYSTEM /opt/homebrew/Cellar/qt@5/5.15.13_1/include)

find_package(Boost REQUIRED)
find_package(anl REQUIRED)

# Main Application

function(filter_files source_list pattern dest_list)
    set(filtered_list "")
    foreach (file ${${source_list}})
        if (NOT file MATCHES ${pattern})
            list(APPEND filtered_list ${file})
        endif ()
    endforeach ()
    set(${dest_list} ${filtered_list} PARENT_SCOPE)
endfunction()

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

##########

#cmake_minimum_required(VERSION 3.26)
#project(GlobeArt CXX)
#
#set(CMAKE_CXX_STANDARD 20)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)
#set(CMAKE_VERBOSE_MAKEFILE ON)
#set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE true)
#
##set(CMAKE_EXPERIMENTAL_CXX_MODULE_CMAKE_API "2182bf5c-ef0d-489a-91da-49dbc3090d2a")
#set(CMAKE_CXX_SCAN_FOR_MODULES ON)
#
#if (
#(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "11.0") OR
#(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "13.0")
#)
#    message(FATAL_ERROR "GCC 11 or higher or Clang 13 or higher is required for C++20 modules.")
#endif ()
#
## Libraries
#
#set(Qt5_DIR "/opt/homebrew/opt/qt@5/lib/cmake/Qt5")
#
#find_package(PkgConfig REQUIRED)
#find_package(CGAL QUIET COMPONENTS Qt5)
#add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
#include_directories(BEFORE SYSTEM /opt/homebrew/Cellar/qt@5/5.15.13_1/include)
#find_package(Boost REQUIRED)
#find_package(anl REQUIRED)
#
#function(enable_modules target)
#    if(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
#        target_compile_options(${target} PRIVATE -std=c++20 -fmodules -fcxx-modules)
#    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
#        target_compile_options(${target} PRIVATE -std=c++20 -fmodules-ts)
#    endif()
#endfunction()
#
#set(
#        GLOBE_HEADERS
#        src/globe/globe_viewer.hpp
#        src/globe/globe_generator.hpp
#        src/globe/point_generator/point_generator.hpp
#        src/globe/point_generator/mock_point_generator.hpp
#        src/globe/point_generator/random_sphere_point_generator.hpp
#        src/globe/sphere_mesh_generator/sphere_mesh_generator.hpp
#        src/globe/noise_generator/point_iterator.hpp
#        src/globe/noise_generator/range.hpp
#        src/globe/noise_generator/anl_noise_generator.hpp
#        src/globe/noise_generator/noise_generator.hpp
#        src/globe/geometry/helpers.hpp
#        src/globe/types.hpp
#)
#
#set(
#        GLOBE_SOURCES
#        src/generate_globe.cpp
#        src/globe/point_generator/random_sphere_point_generator.cpp
#        src/globe/points_collection/points_collection.cpp
#        src/globe/globe_viewer.cpp
#        src/globe/noise_generator/range.cpp
#        src/globe/noise_generator/anl_implementation.cpp
#        src/globe/noise_generator/point_iterator.cpp
#)
#
#set(
#        GLOBE_MODULES
#        src/globe/points_collection/types.ixx
#        src/globe/points_collection/handle_iterator.ixx
#        src/globe/points_collection/points_collection.ixx
#)
#
## Modules
#add_library(globe_modules OBJECT)
#target_sources(globe_modules PRIVATE FILE_SET CXX_MODULES TYPE CXX_MODULES FILES ${GLOBE_MODULES})
#target_include_directories(globe_modules PRIVATE ${CGAL_INCLUDE_DIRS})
#enable_modules(globe_modules)
#
## Main Application
#add_executable(generate_globe ${GLOBE_HEADERS} ${GLOBE_SOURCES})
#target_sources(generate_globe PRIVATE FILE_SET CXX_MODULES TYPE CXX_MODULES FILES ${GLOBE_MODULES})
#target_include_directories(generate_globe PRIVATE ${QT5_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS})
#target_link_libraries(generate_globe PRIVATE CGAL::CGAL CGAL::CGAL_Qt5 anl $<TARGET_OBJECTS:globe_modules>)
#target_compile_definitions(generate_globe PRIVATE CGAL_USE_BASIC_VIEWER QT_NO_KEYWORDS)
#enable_modules(generate_globe)

