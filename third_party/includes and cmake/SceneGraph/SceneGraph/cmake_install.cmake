# Install script for directory: /home/algomorph/Garage/SceneGraph/SceneGraph

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libscenegraph.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libscenegraph.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libscenegraph.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libscenegraph.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/home/algomorph/Garage/SceneGraph/SceneGraph/libscenegraph.so")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libscenegraph.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libscenegraph.so")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}/usr/local/lib/libscenegraph.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libscenegraph.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/SceneGraph" TYPE FILE FILES "/home/algomorph/Garage/SceneGraph/SceneGraph/SceneGraphConfigVersion.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/SceneGraph/SceneGraph.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/SceneGraph/SceneGraph.cmake"
         "/home/algomorph/Garage/SceneGraph/SceneGraph/CMakeFiles/Export/lib/cmake/SceneGraph/SceneGraph.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/SceneGraph/SceneGraph-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/SceneGraph/SceneGraph.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/SceneGraph" TYPE FILE FILES "/home/algomorph/Garage/SceneGraph/SceneGraph/CMakeFiles/Export/lib/cmake/SceneGraph/SceneGraph.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/SceneGraph" TYPE FILE FILES "/home/algomorph/Garage/SceneGraph/SceneGraph/CMakeFiles/Export/lib/cmake/SceneGraph/SceneGraph-release.cmake")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/SceneGraph" TYPE FILE FILES "/home/algomorph/Garage/SceneGraph/SceneGraph/CMakeFiles/SceneGraphConfig.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/SceneGraph/config.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/SceneGraph" TYPE FILE FILES "/home/algomorph/Garage/SceneGraph/SceneGraph/config.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/SceneGraph/config.h.in;/usr/local/include/SceneGraph/SceneGraph.h;/usr/local/include/SceneGraph/GLHelpers.h;/usr/local/include/SceneGraph/GLSLHelpers.h;/usr/local/include/SceneGraph/GLSceneGraph.h;/usr/local/include/SceneGraph/GLHeightmap.h;/usr/local/include/SceneGraph/GLObject.h;/usr/local/include/SceneGraph/GLGroup.h;/usr/local/include/SceneGraph/GLCylinder.h;/usr/local/include/SceneGraph/GLLight.h;/usr/local/include/SceneGraph/GLShadowLight.h;/usr/local/include/SceneGraph/GLColor.h;/usr/local/include/SceneGraph/GLImage.h;/usr/local/include/SceneGraph/GLVbo.h;/usr/local/include/SceneGraph/GLText.h;/usr/local/include/SceneGraph/GLGrid.h;/usr/local/include/SceneGraph/GLAxis.h;/usr/local/include/SceneGraph/GLCube.h;/usr/local/include/SceneGraph/GLTeapot.h;/usr/local/include/SceneGraph/GLLineStrip.h;/usr/local/include/SceneGraph/GLPrimitives.h;/usr/local/include/SceneGraph/GLAxisHistory.h;/usr/local/include/SceneGraph/GLOpenBox.h;/usr/local/include/SceneGraph/GLHelpersLoadTextures.h;/usr/local/include/SceneGraph/GLHelpersBoost.h;/usr/local/include/SceneGraph/GLHelpersDevil.h;/usr/local/include/SceneGraph/GLWireSphere.h;/usr/local/include/SceneGraph/AxisAlignedBoundingBox.h;/usr/local/include/SceneGraph/LineSegment.h;/usr/local/include/SceneGraph/PangolinDrawGLObject.h;/usr/local/include/SceneGraph/PangolinSceneGraphHandler.h;/usr/local/include/SceneGraph/PangolinImageView.h;/usr/local/include/SceneGraph/GLMovableAxis.h;/usr/local/include/SceneGraph/GLAxisAlignedBox.h;/usr/local/include/SceneGraph/GLWaypoint.h;/usr/local/include/SceneGraph/FBO.h;/usr/local/include/SceneGraph/SimCam.h;/usr/local/include/SceneGraph/nvWidgets.h;/usr/local/include/SceneGraph/nvGlutWidgets.h;/usr/local/include/SceneGraph/nvGLWidgets.h;/usr/local/include/SceneGraph/nvShaderUtils.h;/usr/local/include/SceneGraph/GLWidgetView.h;/usr/local/include/SceneGraph/GLMesh.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/SceneGraph" TYPE FILE FILES
    "/home/algomorph/Garage/SceneGraph/SceneGraph/config.h.in"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/SceneGraph.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLHelpers.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLSLHelpers.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLSceneGraph.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLHeightmap.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLObject.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLGroup.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLCylinder.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLLight.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLShadowLight.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLColor.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLImage.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLVbo.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLText.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLGrid.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLAxis.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLCube.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLTeapot.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLLineStrip.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLPrimitives.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLAxisHistory.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLOpenBox.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLHelpersLoadTextures.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLHelpersBoost.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLHelpersDevil.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLWireSphere.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/AxisAlignedBoundingBox.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/LineSegment.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/PangolinDrawGLObject.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/PangolinSceneGraphHandler.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/PangolinImageView.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLMovableAxis.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLAxisAlignedBox.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLWaypoint.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/FBO.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/SimCam.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/../Widgets/nvWidgets.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/../Widgets/nvGlutWidgets.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/../Widgets/nvGLWidgets.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/../Widgets/nvShaderUtils.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/../Widgets/GLWidgetView.h"
    "/home/algomorph/Garage/SceneGraph/SceneGraph/GLMesh.h"
    )
endif()

