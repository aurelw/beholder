include_directories (../common/)
include_directories (../core/)

set (calib_source_files calibstorage_contract.cpp)
add_library (calib ${calib_source_files})

#add_executable (ftrack ftrack.cpp)
#target_link_libraries (gui )

