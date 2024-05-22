cmake_minimum_required(VERSION 3.1...3.23)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(GlobeArt)

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

add_executable(surprise src/experiments/surprise.cpp)
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
