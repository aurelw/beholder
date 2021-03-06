include_directories (../common/)
include_directories (../core/)
include_directories (../gui/)
include_directories (../calib/)

set (EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin/tests/")

### RigConfig Test
set (test_rig_config_source_files test_rig_config.cpp)
add_executable (test_rig_config ${test_rig_config_source_files})
target_link_libraries (test_rig_config core)


### Core Pullup Test
set (test_core_pullup_source_files test_core_pullup.cpp)
add_executable (test_core_pullup ${test_core_pullup_source_files})
target_link_libraries (test_core_pullup core)

# copy rig config
configure_file ("rigconfig_core_pullup.xml" "${EXECUTABLE_OUTPUT_PATH}/rigconfig_core_pullup.xml" COPYONLY)


### FocusController Test
set (test_focuscontroller_source_files test_focuscontroller.cpp)
add_executable (test_focuscontroller ${test_focuscontroller_source_files})
target_link_libraries (test_focuscontroller core)


### VideoStream Test
set (test_videostream_source_files test_videostream.cpp)
add_executable (test_videostream ${test_videostream_source_files})
target_link_libraries (test_videostream core)

# copy rig config
configure_file ("rigconfig_videostream.xml" "${EXECUTABLE_OUTPUT_PATH}/rigconfig_videostream.xml" COPYONLY)


### CameraInterface Test
set (test_camerainterface_source_files test_camerainterface.cpp)
add_executable (test_camerainterface ${test_camerainterface_source_files})
target_link_libraries (test_camerainterface core)

# copy rig config
configure_file ("rigconfig_camerainterface.xml" "${EXECUTABLE_OUTPUT_PATH}/rigconfig_camerainterface.xml" COPYONLY)


### CalibVisualizer Test
set (test_calibvisualizer_source_files test_calibvisualizer.cpp)
add_executable (test_calibvisualizer ${test_calibvisualizer_source_files})
target_link_libraries (test_calibvisualizer core gui)

### estimateAffine3D Test
set (test_estimateaffine_files test_estimateaffine.cpp)
add_executable (test_estimateaffine ${test_estimateaffine_files})
target_link_libraries (test_estimateaffine core calib)

### Convert Affine Test
set (test_convertaffine_files test_convertaffine.cpp)
add_executable (test_convertaffine ${test_convertaffine_files})
target_link_libraries (test_convertaffine core)

### Octave Interface Test
set (test_octave_files test_octave.cpp)
add_executable (test_octave ${test_octave_files})
target_link_libraries (test_octave core calib)

### alglib Test
set (test_alglib_files test_alglib.cpp)
add_executable (test_alglib ${test_alglib_files})
target_link_libraries (test_alglib core calib ${ALGLIB_LIBRARIES})

### GSL Test
set (test_gsl_files test_gsl.cpp)
add_executable (test_gsl ${test_gsl_files})
target_link_libraries (test_gsl core calib ${GSL_LIBRARIES})

