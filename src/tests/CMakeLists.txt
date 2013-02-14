include_directories (../common/)
include_directories (../core/)
include_directories (../gui/)

# RigConfig Test
set (test_rig_config_source_files test_rig_config.cpp)
add_executable (test_rig_config ${test_rig_config_source_files})
target_link_libraries (test_rig_config core)

# Core Pullup Test
set (test_core_pullup_source_files test_core_pullup.cpp)
add_executable (test_core_pullup ${test_core_pullup_source_files})
target_link_libraries (test_core_pullup core)

# Core Pullup Test
set (test_focuscontroller_source_files test_focuscontroller.cpp)
add_executable (test_focuscontroller ${test_focuscontroller_source_files})
target_link_libraries (test_focuscontroller core)
