include_directories (../common/)
include_directories (../core/)

set (gui_source_files ffocus_visualizer.cpp calib_visualizer.cpp)
add_library (gui ${gui_source_files})

#add_executable (ftrack ftrack.cpp)
#target_link_libraries (gui )

