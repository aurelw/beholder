cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(beholder)

# compiler options
add_definitions("-std=gnu++11")

# options
option (BUILD_APP "Build with application frontend." ON)
option (BUILD_TESTS "Build with tests." ON)
set(GPHOTO_BIN "" CACHE FILEPATH "Path to gPhoto binary.")

# deal with pcl
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# deal with boost
find_package(Boost COMPONENTS signals filesystem serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIR} )

# OpenCV
find_package( OpenCV 2.3 REQUIRED )

# ImageMagick
find_package(ImageMagick COMPONENTS Magick++)
include_directories(${ImageMagick_INCLUDE_DIRS} )

# Octave
set (CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/)
find_package(Octave)
include_directories(${OCTAVE_INCLUDE_DIR})
link_directories(${OCTAVE_LIBDIR})

# ZMQ
#find_package(ZeroMQ REQUIRED)

# LIBALG
find_package( alglib REQUIRED )
link_directories(${ALGLIB_LIBRARY_DIRS})
include_directories(${ALGLIB_INCLUDE_DIRS})

# GSL
find_package( GSL )
link_directories(${GSL_LIBRARY_DIRS})
include_directories(${GSL_INCLUDE_DIRS})


### configure ###
set (BEHOLDER_VERSION 0.1)
set (BH_DEBUG_LVL 4)

configure_file (
    "${PROJECT_SOURCE_DIR}/src/common/beholder_config.h.in"
    "${PROJECT_SOURCE_DIR}/src/common/beholder_config.h"
)
#################

# specify output directories
make_directory ("${CMAKE_BINARY_DIR}/bin/")
make_directory ("${CMAKE_BINARY_DIR}/bin/tests/")
make_directory ("${CMAKE_BINARY_DIR}/bin/tools/")
set (EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin") 

add_subdirectory(src)

