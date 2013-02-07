include_directories (../common/)

set (core_source_files kinfutrackerwrap.cpp capture.cpp cameraparameters.cpp ffocus_messure.cpp focusmotor.cpp focustracker.cpp focustracker_interpolate.cpp focustracker_multi.cpp focustracker_nearest.cpp focustracker_single.cpp viewfinder.cpp rig_config.cpp motor.cpp bytemotor.cpp motor_factory.cpp testmotor.cpp focusmotor_twopolys.cpp mathutils.cpp)

add_library (core ${core_source_files})
target_link_libraries (core ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

