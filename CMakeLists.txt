cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(beholder)

# options
option (BUILD_APP "Build with application frontend." ON)
option (BUILD_TESTS "Build with tests." ON)

# deal with pcl
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# deal with boost
find_package(Boost COMPONENTS signals filesystem serialization)
include_directories(${Boost_INCLUDE_DIR} )

# OpenCV
find_package( OpenCV REQUIRED )

### configure ###
set (BEHOLDER_VERSION 0.1)
set (BH_DEBUG_LVL 4)

configure_file (
    "${PROJECT_SOURCE_DIR}/src/common/beholder_config.h.in"
    "${PROJECT_SOURCE_DIR}/src/common/beholder_config.h"
)
#################

add_subdirectory(src)

