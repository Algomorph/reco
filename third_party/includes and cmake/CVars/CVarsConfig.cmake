SET( CVars_LIBRARIES  /usr/local/lib/libcvars.a CACHE INTERNAL "CVars libraries" FORCE )
SET( CVars_INCLUDE_DIRS   CACHE INTERNAL "CVars include directories" FORCE )
SET( CVars_LIBRARY_DIRS  CACHE INTERNAL "CVars library directories" FORCE )

mark_as_advanced( CVars_LIBRARIES )
mark_as_advanced( CVars_LIBRARY_DIRS )
mark_as_advanced( CVars_INCLUDE_DIRS )



# Compute paths
get_filename_component( PACKAGE_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )


# Library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET "CVars_LIBRARIES" AND NOT "CVars_BINARY_DIR")
    include( "${PACKAGE_CMAKE_DIR}/CVarsTargets.cmake" )
    include( "${PACKAGE_CMAKE_DIR}/CVarsConfigVersion.cmake" )
endif()

#SET(CVars_LIBRARIES )
#SET(CVars_LIBRARY )
#SET(CVars_INCLUDE_DIRS )
#SET(CVars_LINK_DIRS )
