#Applications CMakeLists.txt

### ============================================================================
###                              FIND PACKAGES
### ============================================================================

# Freenect
find_package (freenect2 REQUIRED)
#temp fix for OpenKinect/libfreenect2 issue #217
set(freenect2_INCLUDE_DIRS ${freenect2_INCLUDE_DIR} ${freenect2_INCLUDE_DIR}/libfreenect2/tinythread)
include_directories(${freenect2_INCLUDE_DIRS})

# OpenCV
find_package (OpenCV REQUIRED core highgui calib3d imgproc)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_INSTALL_PATH})

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Boost
find_package(Boost REQUIRED COMPONENTS regex system filesystem) 

### ARPG dependencies-----------------------------------------------------------

find_package(SceneGraph 0.1 REQUIRED)
include_directories(${SceneGraph_INCLUDE_DIRS})

find_package(Pangolin 0.1 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(HAL 0.1 REQUIRED)
include_directories( ${HAL_INCLUDE_DIRS} )

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

find_package(Calibu 0.1 REQUIRED)
include_directories(${Calibu_INCLUDE_DIRS})

find_package(Kangaroo 0.1 REQUIRED)
include_directories(${Kangaroo_INCLUDE_DIRS})

find_package(Boost COMPONENTS system filesystem regex)

include_directories(${CMAKE_SOURCE_DIR}/include/alex)


### ----------------------------------------------------------------------------
### C++11 support
### ----------------------------------------------------------------------------

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


### ----------------------------------------------------------------------------
### Executables
### ----------------------------------------------------------------------------

# Built-in Parameter Extraction
add_executable(extract-params intrinsics.cpp)
target_link_libraries(extract-params ${freenect2_LIBRARY} ${OpenCV_LIBS})

# Data capture
add_executable( capture capture.cpp )
target_link_libraries(capture dl 
    ${Boost_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${HAL_LIBRARIES} 
)

# Data extraction & optional undistort
add_executable( extract extract.cpp )
target_link_libraries(extract dl ${HAL_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

# Captured data playback
add_executable( playback playback.cpp )
target_link_libraries(playback  
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${Calibu_LIBRARIES}
)
                                      
# Kinfu library
#TODO: fix                                      
#add_library(kinfu SHARED kinfu.cpp)
#target_link_libraries(kinfu  ${PCL_LIBRARIES}
#                             ${Boost_LIBRARIES}
#                             ${OpenCV_LIBS}
#                                kinfu)
#TODO: fix
# Kinfu app
#add_executable( kinfuapp kinfu_app.cpp )
#target_link_libraries(kinfuapp  kinfu)