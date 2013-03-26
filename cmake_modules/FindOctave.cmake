# - Try to find Octave
# Once done this will define
#  OCTAVE_FOUND - System has octave
#  OCTAVE_INCLUDE_DIRS - The octave include directories
#  OCTAVE_LIBRARIES - The libraries needed to use octave
#  OCTAVE_LIBDIR - Compiler switches required for using octave


execute_process(COMMAND octave-config -p OCTLIBDIR OUTPUT_VARIABLE OCTAVE_LIBDIR OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND octave-config -p OCTINCLUDEDIR OUTPUT_VARIABLE OCTAVE_INCDIR OUTPUT_STRIP_TRAILING_WHITESPACE)

find_path(OCTAVE_INCLUDE_DIR octave/oct.h PATHS ${OCTAVE_INCDIR} ${OCTAVE_INCDIR}/.. )

find_library(OCTAVE_LIBRARY octave PATHS ${OCTAVE_LIBDIR} )
find_library(OCTINTERP_LIBRARY octinterp PATHS ${OCTAVE_LIBDIR} )


IF(OCTAVE_INCLUDE_DIR AND OCTAVE_LIBRARY)
  MESSAGE(STATUS "OCTAVE_INCLUDE_DIR=${OCTAVE_INCLUDE_DIR}")
  MESSAGE(STATUS "OCTAVE_LIBRARY=${OCTAVE_LIBRARY}")
  get_filename_component(OCTAVE_LIBDIR ${OCTAVE_LIBRARY} PATH)
ELSE()
  IF(OCTAVE_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "OCTAVE required, please specify it's location.")
  ELSE()
    MESSAGE(STATUS      "OCTAVE was not found.")
  ENDIF()
ENDIF()



set(OCTAVE_LIBRARIES ${OCTAVE_LIBRARY} ${OCTINTERP_LIBRARY})
set(OCTAVE_INCLUDE_DIRS ${OCTAVE_INCLUDE_DIR} )




# include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBOCTAVE_FOUND to TRUE
# if all listed variables are TRUE
# find_package_handle_standard_args(liboctave  DEFAULT_MSG
#                                   OCTAVE_LIBRARY OCTAVE_INCLUDE_DIR)

# mark_as_advanced(OCTAVE_INCLUDE_DIR OCTAVE_LIBRARY)
