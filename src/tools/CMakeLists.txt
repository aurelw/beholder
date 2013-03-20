include_directories (../common/)
include_directories (../core/)
include_directories (../gui/)
include_directories (../calib/)

set (EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin/tools/")

### Render Randge Image
set (take_range_image_files take_range_image.cpp)
add_executable (take_range_image ${take_range_image_files})
target_link_libraries (take_range_image core)

### Undistort Image
set (undistort_image_files undistort_image.cpp)
add_executable (undistort_image ${undistort_image_files})
target_link_libraries (undistort_image core)

### Show Correspondences
set (show_correspondences_files show_correspondences.cpp)
add_executable (show_correspondences ${show_correspondences_files})
target_link_libraries (show_correspondences core calib gui)

### Contrast level in VideoStream
set (contrast_level_files contrast_level.cpp)
add_executable (contrast_level ${contrast_level_files})
target_link_libraries (contrast_level core calib gui)

### Contrast level in VideoStream
set (motor_control_files motor_control.cpp)
add_executable (motor_control ${motor_control_files})
target_link_libraries (motor_control core calib gui)

