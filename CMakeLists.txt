cmake_minimum_required(VERSION 3.1...3.23)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(GlobeArt)

# Libraries

find_package(PkgConfig REQUIRED)

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE true) # remove for release
find_package(CGAL QUIET COMPONENTS Qt5)
find_package(Boost REQUIRED)

set(ENV{PKG_CONFIG_PATH} "/opt/homebrew/opt/qt@5/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
pkg_search_module(QT5 REQUIRED Qt5Widgets)

pkg_search_module(PNG REQUIRED libpng)
find_library(PNG_LIBRARY_FULLPATH NAMES ${PNG_LIBRARIES} PATHS ${PNG_LIBRARY_DIRS} NO_DEFAULT_PATH)

find_library(NOISE_LIBRARY NAMES libnoise noise)
find_path(NOISE_INCLUDE_DIR NAMES noise/noise.h)
find_package(tinyply REQUIRED)
find_package(anl REQUIRED)

# Experiments

add_executable(surprise src/experiments/surprise.cpp src/experiments/test_mesh_capabilities.cpp
        src/experiments/test_union.cpp)

target_include_directories(surprise PRIVATE ${QT5_INCLUDE_DIRS})
target_link_libraries(surprise PRIVATE CGAL::CGAL_Qt5)
target_compile_definitions(surprise PRIVATE CGAL_USE_BASIC_VIEWER QT_NO_KEYWORDS)

add_executable(test_cgal src/experiments/test_cgal.cpp)
target_link_libraries(test_cgal PRIVATE CGAL::CGAL)

add_executable(test_libnoise_png src/experiments/test_libnoise_png.cpp)
target_include_directories(test_libnoise_png PRIVATE ${PNG_INCLUDE_DIRS} ${NOISE_INCLUDE_DIR})
target_link_libraries(test_libnoise_png PRIVATE ${PNG_LIBRARY_FULLPATH} ${NOISE_LIBRARY})

add_executable(test_ply src/experiments/test_ply.cpp)
target_link_libraries(test_ply PRIVATE tinyply)

add_executable(test_cgal_sphere src/experiments/test_cgal_sphere.cpp)
target_include_directories(test_cgal_sphere PRIVATE ${NOISE_INCLUDE_DIR})
target_link_libraries(test_cgal_sphere PRIVATE CGAL::CGAL ${NOISE_LIBRARY})

add_executable(test_anl src/experiments/test_anl.cpp)
target_link_libraries(test_anl PRIVATE anl)

add_executable(test_mesh_capabilities src/experiments/test_mesh_capabilities.cpp)
target_link_libraries(test_mesh_capabilities PRIVATE CGAL::CGAL)

add_executable(test_union src/experiments/test_union.cpp)
target_link_libraries(test_union PRIVATE CGAL::CGAL)

# Generate Globe

function(filter_files source_list pattern dest_list) # custom function to filter out files based on a pattern
    set(filtered_list "")
    foreach(file ${${source_list}})
        if(NOT file MATCHES ${pattern})
            list(APPEND filtered_list ${file})
        endif()
    endforeach()
    set(${dest_list} ${filtered_list} PARENT_SCOPE)
endfunction()

file(GLOB_RECURSE GLOBE_HEADERS src/globe/*.h)
file(GLOB_RECURSE GLOBE_SOURCES src/globe/*.cpp)
filter_files(GLOBE_SOURCES "_test.cpp$" GLOBE_SOURCES)

add_executable(generate_globe src/generate_globe.cpp ${GLOBE_HEADERS} ${GLOBE_SOURCES})

target_include_directories(generate_globe PRIVATE ${NOISE_INCLUDE_DIR})
target_link_libraries(generate_globe PRIVATE CGAL::CGAL ${NOISE_LIBRARY})

# Tests

pkg_search_module(GTEST REQUIRED gtest_main)
file(GLOB_RECURSE TEST_SOURCES src/globe/*_test.cpp)

add_executable(test_project)
target_sources(test_project PRIVATE ${TEST_SOURCES})
target_link_libraries(test_project PRIVATE ${GTEST_LDFLAGS})
target_compile_options(test_project PRIVATE ${GTEST_CFLAGS})

enable_testing()
add_test(first_and_only_test testapp)
