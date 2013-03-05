include_directories (../common/)
include_directories (../core/)
include_directories (../gui/)

set (calib_source_files calibstorage_contract.cpp calib_utils.cpp)
add_library (calib ${calib_source_files})
target_link_libraries(calib core)


#### tools and exexutables ####

# add_intrinsic_samples
set (add_intrinsic_samples_file add_intrinsic_samples.cpp)
add_executable (add_intrinsic_samples ${add_intrinsic_samples_file})
target_link_libraries (add_intrinsic_samples core calib)

# capture_extrinsic_pair
set (capture_extrinsic_pair_files capture_extrinsic_pair.cpp)
add_executable (capture_extrinsic_pair ${capture_extrinsic_pair_files})
target_link_libraries (capture_extrinsic_pair core calib gui)

# find_marker_pair
set (find_marker_pair_files find_marker_pair.cpp)
add_executable (find_marker_pair ${find_marker_pair_files})
target_link_libraries (find_marker_pair core calib gui)

# calibrate_extrinsic_pnp
set (calibrate_extrinsic_pnp_files calibrate_extrinsic_pnp.cpp)
add_executable (calibrate_extrinsic_pnp ${calibrate_extrinsic_pnp_files})
target_link_libraries (calibrate_extrinsic_pnp core calib)

# calibrate_intrinsic
set (calibrate_intrinsic_files calibrate_intrinsic.cpp)
add_executable (calibrate_intrinsic ${calibrate_intrinsic_files})
target_link_libraries (calibrate_intrinsic core calib)

# calibrate_intrinsic
set (calibrate_extrinsic_3dreg_files calibrate_extrinsic_3dreg.cpp)
add_executable (calibrate_extrinsic_3dreg ${calibrate_extrinsic_3dreg_files})
target_link_libraries (calibrate_extrinsic_3dreg core calib)
