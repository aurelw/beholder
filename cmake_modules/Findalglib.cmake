# - Try to find agllib 
# Once done this will define
#  FOUND_ALGLIB - System has AGL
#  ALGLIB_INCLUDE_DIRS - The octave include directories
#  ALGLIB_LIBRARIES - The libraries needed to use octave
#  ALGBLIB_LIBDIR - Compiler switches required for using octave


set (ALGLIB_INCLUDE_DIR "/usr/include/")

set (ALGLIB_LIBDIR "/usr/lib/")
find_library(ALGLIB_LIBRARY alglib PATHS ${ALGLIB_LIBDIR} )

IF(OCTAVE_INCLUDE_DIR AND OCTAVE_LIBRARY)
    MESSAGE(STATUS "LIBALG_INCLUDE_DIR=${ALGLIB_INCLUDE_DIR}")
    MESSAGE(STATUS "LIBALG_LIBRARY=${ALGLIB_LIBRARY}")
    get_filename_component(ALGLIB_LIBDIR ${ALGLIB_LIBRARY} PATH)
ELSE()
    IF(ALGLIB_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "libalg required, please specify it's location.")
  ELSE()
    MESSAGE(STATUS      "libalg was not found.")
  ENDIF()
ENDIF()

set(LIBALG_LIBRARIES ${LIBALG_LIBRARY}})
set(LIBALG_INCLUDE_DIRS ${LIBALG_INCLUDE_DIR})

