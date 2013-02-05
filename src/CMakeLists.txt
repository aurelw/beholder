add_subdirectory (core)
add_subdirectory (gui)
add_subdirectory (common)
add_subdirectory (calib)
add_subdirectory (app)
add_subdirectory (tests)

include_directories (./core/)
include_directories (./gui/)
include_directories (./common/)

#add_executable (weight_tracker weighttracker.cpp barabella_app.cpp)
#target_link_libraries (weight_tracker gui)

