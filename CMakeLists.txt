cmake_minimum_required(VERSION 3.1...3.23)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(ProjectName)

find_package(PkgConfig REQUIRED)

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE true) # remove for release
find_package(CGAL QUIET COMPONENTS Qt5)
find_package(Boost REQUIRED)

set(ENV{PKG_CONFIG_PATH} "/opt/homebrew/opt/qt@5/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
pkg_search_module(QT5 REQUIRED Qt5Widgets)

pkg_search_module(PNG REQUIRED libpng)
find_library(PNG_LIBRARY_FULLPATH NAMES ${PNG_LIBRARIES} PATHS ${PNG_LIBRARY_DIRS} NO_DEFAULT_PATH)

add_executable(surprise surprise.cpp)
target_include_directories(surprise PRIVATE ${QT5_INCLUDE_DIRS})

target_link_libraries(surprise PRIVATE CGAL::CGAL_Qt5)
target_compile_definitions(surprise PRIVATE CGAL_USE_BASIC_VIEWER QT_NO_KEYWORDS)

add_executable(triang_on_sphere triang_on_sphere.cpp)
target_link_libraries(triang_on_sphere PRIVATE CGAL::CGAL)

add_executable(test_png test_png.cpp)
target_include_directories(test_png PRIVATE ${PNG_INCLUDE_DIRS})
target_link_libraries(test_png PRIVATE ${PNG_LIBRARY_FULLPATH})
