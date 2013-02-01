include_directories (../common/)
include_directories (../core/)
include_directories (../gui/)

set (app_source_files ffocus_app.cpp)

add_executable (ftrack ${app_source_files})
target_link_libraries (ftrack gui core)

