cmake_minimum_required(VERSION 2.8)

project(reco)
set (CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

### ----------------------------------------------------------------------------
### OpenCV dependent options
### ----------------------------------------------------------------------------

find_package (OpenCV REQUIRED core highgui calib3d imgproc)
message (STATUS "")
if (OpenCV_FOUND)
  message (STATUS " --------------------------")
  message (STATUS " Found OpenCV version ${OpenCV_VERSION}")
  message (STATUS "     * include dirs: ${OpenCV_INCLUDE_DIRS}")
  message (STATUS "     * library dirs: ${OpenCV_INSTALL_PATH}")
  message (STATUS "     * libraries:    ${OpenCV_LIBS}")
else()
  message (FATAL_ERROR : "OpenCV NOT FOUND!")
endif()
message (STATUS "")

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_INSTALL_PATH})
link_libraries(${OpenCV_LIBS})

### ----------------------------------------------------------------------------
### PCL
### ----------------------------------------------------------------------------

find_package(PCL REQUIRED)
message (STATUS "")
if (PCL_FOUND)
  message (STATUS "")
  message (STATUS " -----------------------")
  message (STATUS " Found PCL version ${PCL_VERSION}")
  message (STATUS "     * include dirs: ${PCL_INCLUDE_DIRS}")
  message (STATUS "     * library dirs: ${PCL_LIBRARY_DIRS}")
  message (STATUS "     * libraries:    ${PCL_LIBRARIES}")
else()
  message (FATAL_ERROR : "PCL NOT FOUND!")
endif()
message (STATUS "")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

### ----------------------------------------------------------------------------
### BOOST
### ----------------------------------------------------------------------------

find_package(Boost REQUIRED COMPONENTS regex system filesystem) 

if(Boost_FOUND)
  message (STATUS " --------------------------")
  message (STATUS " Found Boost version ${Boost_VERSION}")
  message (STATUS "     * include dirs: ${Boost_INCLUDE_DIRS}")
  message (STATUS "     * libraries:    ${Boost_LIBRARIES}")
endif()

### ----------------------------------------------------------------------------
### ARPG dependencies
### ----------------------------------------------------------------------------

find_package(SceneGraph 0.1 REQUIRED)
include_directories(${SceneGraph_INCLUDE_DIRS})
link_libraries(${SceneGraph_LIBRARIES})

find_package(Pangolin 0.1 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
link_libraries(${Pangolin_LIBRARIES})

find_package(HAL 0.1 REQUIRED)
include_directories( ${HAL_INCLUDE_DIRS} )
link_libraries(${HAL_LIBRARIES})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

find_package(Calibu 0.1 REQUIRED)
include_directories(${Calibu_INCLUDE_DIRS})
link_libraries(${Calibu_LIBRARIES})

find_package(Kangaroo 0.1 REQUIRED)
include_directories(${Kangaroo_INCLUDE_DIRS})
link_libraries(${Kangaroo_LIBRARIES})

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

# Data capture
add_executable( capture src/capture.cpp )
target_link_libraries(capture dl ${Boost_LIBRARIES})

# Data extraction & optional undistort
add_executable( extract src/extract.cpp )
target_link_libraries(extract dl ${HAL_LIBRARIES} ${Boost_LIBRARIES})

# Captured data playback
add_executable( playback src/playback.cpp )
target_link_libraries(playback  ${PCL_LIBRARIES}
                                ${Boost_LIBRARIES})
                                      
# Kinfu library                                      
add_library(kinfu SHARED src/kinfu.cpp)

# Kinfu app
add_executable( kinfuapp src/kinfu_app.cpp )
target_link_libraries(kinfuapp  ${PCL_LIBRARIES}
                                ${Boost_LIBRARIES}
                                kinfu)