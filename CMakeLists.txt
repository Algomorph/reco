#===============Reco top-level CMakeLists.txt==========================#  
#
#  Created on: Dec 8, 2014
#      Author: Gregory Kramida
#     License: Apache v2
#   Copyright: (c) Gregory Kramida 2014, 2015
#
# EDITING GUIDELINES:
# 1) Use lowercase CMake commands
# 2) Succinct > descriptive > abbreviated
# 3) Less = more
# 4) Hard now = easy later
# 5) Freedom = slavery
#

cmake_minimum_required(VERSION 2.8.11)
	
#----------------------------PROJECT NAME--------------------------------------#
project(reco)
set(global_project_name "${PROJECT_NAME}")
string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)

#----------------------------CMAKE & GLOBAL PROPERTIES-------------------------#
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(CMAKE_MAJOR_VERSION GREATER 2 AND CMAKE_MINOR_VERSION GREATER 0)
    cmake_policy(SET CMP0054 NEW)#no implicit dereferencing for STREQUAL
    cmake_policy(SET CMP0053 NEW)#no special characters in variable names
endif()

#-------------------------INCLUDE UTILITIES------------------------------------#
include("RecoUtilities")

#----------------------------BUILD CONFIGURATIONS------------------------------#
set(DEBUG_BUILD_TYPE_OPTIONS Debug RelWithDebInfo)
set(RELEASE_BUILD_TYPE_OPTIONS Release MinSizeRel)
set(BUILD_TYPE_OPTIONS ${DEBUG_BUILD_TYPE_OPTIONS} ${RELEASE_BUILD_TYPE_OPTIONS})

#set build type 
if(NOT MSVC)#TODO: also don't do for XCode generator?
    #work around CMakes strange CACHE setting system
    if("${CMAKE_BUILD_TYPE}" STREQUAL "")
        set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: ${BUILD_TYPE_OPTIONS}." FORCE)
    else()
        set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: ${BUILD_TYPE_OPTIONS}.")
    endif()
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${BUILD_TYPE_OPTIONS})
    set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE})
endif()

#----------------------------COMPILER FLAGS-----------------------------------#
if(MSVC)
	#ADD_DEFINITIONS(-DUNICODE -D_UNICODE) #TODO: not sure about those
else()
	set(CMAKE_CXX_FLAGS "-Wall")
	if(DEFINED CMAKE_BUILD_TYPE)
    	if(${CMAKE_BUILD_TYPE} STREQUAL "Debug" OR ${CMAKE_BUILD_TYPE} STREQUAL "RelWithDebInfo")
    	    add_definitions(-D_DEBUG) #add the debug flag
    	else()
    	    add_definitions(-DQT_NO_DEBUG_OUTPUT) #add the qt debug output flag
    	endif()
    endif() 
    
    ### C++11 support
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
endif()

#----------------------------OUTPUT DIRECTORIES--------------------------------#
#generic no-config case (e.g. with gcc/mingw)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

#multi-config builds (e.g. msvc)
foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${CMAKE_BINARY_DIR}/bin")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${CMAKE_BINARY_DIR}/lib")
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${CMAKE_BINARY_DIR}/lib")
endforeach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})

#----------------------------NON-COMPILE SOURCE FILES--------------------------#
file(GLOB_RECURSE ${PROJECT_NAME_UPPER}_CMAKE ${CMAKE_SOURCE_DIR}/cmake/*.cmake)
file(GLOB ${PROJECT_NAME_UPPER}_TOP_CMAKE ${CMAKE_SOURCE_DIR}/CMakeLists.txt)
file(GLOB ${PROJECT_NAME_UPPER}_APPLICATIONS_CMAKE ${CMAKE_SOURCE_DIR}/src/applications/CMakeLists.txt)
file(GLOB ${PROJECT_NAME_UPPER}_MODULES_CMAKE ${CMAKE_SOURCE_DIR}/src/modules/CMakeLists.txt)

source_group("Extra CMake Scripts" FILES ${${PROJECT_NAME_UPPER}_CMAKE})
source_group("Top-level CMake" FILES ${${PROJECT_NAME_UPPER}_TOP_CMAKE})
source_group("Applications CMake" FILES ${${PROJECT_NAME_UPPER}_APPLICATIONS_CMAKE})
source_group("Modules CMake" FILES ${${PROJECT_NAME_UPPER}_MODULES_CMAKE})

set(NON_COMPILE_SOURCES 
    ${${PROJECT_NAME_UPPER}_CMAKE}
    ${${PROJECT_NAME_UPPER}_TOP_CMAKE}
    ${${PROJECT_NAME_UPPER}_MODULES_CMAKE} 
    ${${PROJECT_NAME_UPPER}_SETTINGS} 
)

#add non-compile target
add_custom_target(${PROJECT_NAME}_noncompile SOURCES ${NON_COMPILE_SOURCES})


#-------------------------FIND DEPENDENCIES------------------------------------#
include("ResolveDependencies")


#-------------------------INCLUDE SUBDIRECTORIES-------------------------------#
add_subdirectory ("modules")
add_subdirectory ("applications")

#-------------------------RUN SCRIPTS------------------------------------------#
if(UNIX)
    execute_process(
       COMMAND "python" "./scripts/addEclipseProjectFilter.py" "-p" "${CMAKE_BINARY_DIR}/.project"
       RESULT_VARIABLE FILT_RET
       WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endif()
#------------------------------------------------------------------------------#

