# Compute paths
get_filename_component( PACKAGE_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )
SET( Calibu_INCLUDE_DIRS "/home/algomorph/Garage/Calibu/src/../include;/home/algomorph/Garage/Calibu/src/include;/home/algomorph/Garage/Calibu/src/include/;/usr/include/eigen3;/usr/local/lib/cmake/Sophus/../../../include;" )
SET( Calibu_INCLUDE_DIR  "/home/algomorph/Garage/Calibu/src/../include;/home/algomorph/Garage/Calibu/src/include;/home/algomorph/Garage/Calibu/src/include/;/usr/include/eigen3;/usr/local/lib/cmake/Sophus/../../../include;" )

# Library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET "" AND NOT Calibu_BINARY_DIR)
  include( "${PACKAGE_CMAKE_DIR}/CalibuTargets.cmake" )
endif()

SET(Calibu_LIBRARIES calibu)
SET(Calibu_LIBRARY calibu)
SET(Calibu_INCLUDE_DIRS /home/algomorph/Garage/Calibu/src/../include;/home/algomorph/Garage/Calibu/src/include;/home/algomorph/Garage/Calibu/src/include/;/usr/include/eigen3;/usr/local/lib/cmake/Sophus/../../../include)
SET(Calibu_LINK_DIRS )
