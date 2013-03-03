include_directories (../common/)
include_directories (../core/)
include_directories (../gui/)

set (EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin/tools/")

### Render Randge Image
set (take_range_image_files take_range_image.cpp)
add_executable (take_range_image ${take_range_image_files})
target_link_libraries (take_range_image core)

### Undistort Image
set (undistort_image_files undistort_image.cpp)
add_executable (undistort_image ${undistort_image_files})
target_link_libraries (undistort_image core)
