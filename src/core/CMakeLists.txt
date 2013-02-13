include_directories (../common/)

set (core_source_files kinfuwrapper.cpp capture.cpp cameraparameters.cpp ffocus_messure.cpp focusmotor.cpp rig_config.cpp motor.cpp bytemotor.cpp motor_factory.cpp testmotor.cpp focusmotor_twopolys.cpp mathutils.cpp rangefinder.cpp posetracker.cpp kinfuwrapper.cpp openni_interface.cpp posetrackerkinfu.cpp viewfinder_rangeimage.cpp transfer1d1d_constantpoly.cpp update_signal.cpp rangeimagewriter.cpp focustracker_single.cpp focustracker_multi.cpp focustracker_nearest.cpp focustracker_interpolate.cpp focustracker.cpp pointofinterest.cpp focuscontroller.cpp centralfocuscontrol.cpp focuscontroller_singlepoint.cpp)

#set (core_source_files kinfuwrapper.cpp capture.cpp cameraparameters.cpp ffocus_messure.cpp focusmotor.cpp focustracker.cpp focustracker_interpolate.cpp focustracker_multi.cpp focustracker_nearest.cpp focustracker_single.cpp viewfinder.cpp rig_config.cpp motor.cpp bytemotor.cpp motor_factory.cpp testmotor.cpp focusmotor_twopolys.cpp mathutils.cpp rangefinder.cpp posetracker.cpp kinfuwrapper.cpp openni_interface.cpp posetrackerkinfu.cpp)

add_library (core ${core_source_files})
target_link_libraries (core ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${ImageMagick_LIBRARIES})

