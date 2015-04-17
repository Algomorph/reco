# Compute paths
get_filename_component( PACKAGE_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )
SET( SceneGraph_INCLUDE_DIRS ";/usr/include/eigen3;/usr/include;/usr/include;/usr/include;/usr/include;/usr/include/x86_64-linux-gnu;/usr/local/lib/cmake/Pangolin/../../../include;/usr/include;/usr/include;/usr/include;/usr/include;/usr/include/eigen3" )
SET( SceneGraph_INCLUDE_DIR  ";/usr/include/eigen3;/usr/include;/usr/include;/usr/include;/usr/include;/usr/include/x86_64-linux-gnu;/usr/local/lib/cmake/Pangolin/../../../include;/usr/include;/usr/include;/usr/include;/usr/include;/usr/include/eigen3" )

# Library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET "" AND NOT SceneGraph_BINARY_DIR)
  include( "${PACKAGE_CMAKE_DIR}/SceneGraphTargets.cmake" )
endif()

SET(SceneGraph_LIBRARIES scenegraph)
SET(SceneGraph_LIBRARY scenegraph)
SET(SceneGraph_INCLUDE_DIRS /usr/include/eigen3;/usr/include;/usr/include;/usr/include;/usr/include;/usr/include/x86_64-linux-gnu;/usr/local/lib/cmake/Pangolin/../../../include;/usr/include;/usr/include;/usr/include;/usr/include;/usr/include/eigen3;/home/algomorph/Garage/SceneGraph/SceneGraph/..;/home/algomorph/Garage/SceneGraph/SceneGraph/..)
SET(SceneGraph_LINK_DIRS )
