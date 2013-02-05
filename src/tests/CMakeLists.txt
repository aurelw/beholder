include_directories (../common/)
include_directories (../core/)
include_directories (../gui/)

set (test_rig_config_source_files test_rig_config.cpp)

add_executable (test_rig_config ${test_rig_config_source_files})
target_link_libraries (test_rig_config core)

