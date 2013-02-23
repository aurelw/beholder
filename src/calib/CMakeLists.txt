include_directories (../common/)
include_directories (../core/)

set (calib_source_files calibstorage_contract.cpp)
add_library (calib ${calib_source_files})


#### tools and exexutables ####

set (add_intrinsic_samples_file add_intrinsic_samples.cpp)
add_executable (add_intrinsic_samples ${add_intrinsic_samples_file})
target_link_libraries (add_intrinsic_samples core calib)
