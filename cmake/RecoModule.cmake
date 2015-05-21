####################################################################################################
#  CMake macro that abstracts away adding modules(library subprojects) and applications(executable 
#  subprojects) in CMake, i.e. turns that into a one-liner.
#
#  Syntax: 
#  reco_add_subproject(<subproject_name> [QT] 
#    [[DEPENDENCIES] [names of packages or modules that this subproject depends on...]]
#    [MODULE | APPLICATION] [TEST])
#
#
#  If the QT flag is passed in, this will let the script know that the module uses the qmake moc 
#  compiler (qt resource files, ui files, and custom widgets are supported).
#  If the TEST flag (currently unsupported) is passed in, the macro will generate an additional test 
#  target using GTest.
####################################################################################################

macro(${global_project_name}_add_subproject _name)
    set(subproject_name ${global_project_name}_${_name})
    
    project(${subproject_name})
    
    set(module_type "MODULE")
    set(app_type "APPLICATION")
    
#----------PARSE ARGUMENTS-------------------------------------------------------------------------#
    set(keywords DEPENDENCIES QT ${module_type} ${app_type})
    set(append_depends 0)
    set(_depends)
    set(_qt 0)
    set(_subproject_type 0)

    foreach(arg ${ARGN})
        #check if it's a keyword
        list(FIND keywords ${arg} index)
        if(index EQUAL -1)
            if(append_depends)
                list(FIND keywords ${arg} index)
                if(index EQUAL -1)
                    list(APPEND _depends ${arg})
                else()
                    set(append_depends 0)
                endif()
            endif()
        else()
            #check which keyword
            if("${arg}" STREQUAL "DEPENDENCIES")
                set(append_depends 1)
            elseif("${arg}" STREQUAL "QT")
                set(_qt 1)
            elseif("${arg}" STREQUAL "${module_type}")
                if(_subproject_type)
                    message(FATAL_ERROR "Trying to set module type repeatedly. Already have: '${_subproject type}'. Now parsed: '${arg}'.")
                endif()
                set(_subproject_type ${module_type})
            elseif("${arg}" STREQUAL "${app_type}")
                if(_subproject_type)
                    message(FATAL_ERROR "Trying to set module type repeatedly. Already have: '${_subproject type}'. Now parsed: '${arg}'.")
                endif()
                set(_subproject_type ${app_type})
            endif()
        endif()
    endforeach()
    
    
    if(NOT _subproject_type)
        message(FATAL_ERROR "Type of module not set. Expecting either '${module_type}' or '${app_type}' as argument.")
    endif()
    
#----------CHECK DEPENDENCIES----------------------------------------------------------------------#
    set(_have_depends TRUE)
    set(_unmet_depends)
    foreach(dep ${_depends})
        if(NOT HAVE_${dep})
            set(_have_depends FALSE)
            list(APPEND _unmet_depends ${dep})
        endif()
    endforeach()
    
    if(NOT DEFINED BUILD_${_name})
        SET(BUILD_${_name} ${_have_depends} CACHE BOOL "Build the '${_name}' module.")
    else()
        if(BUILD_${_name} AND NOT ${_have_depends})
            message(FATAL_ERROR "Cannot build module '${_name}', the following dependencies are not met: ${_unmet_depends}")
        endif()
    endif()
    
    if(_qt)
        set(CMAKE_AUTOMOC ON)
    endif()

#---------------------------DEFINE SOURCE FILES----------------------------------------------------#
    SET(${subproject_name}_top_include_dir include/${global_project_name}/${subproject_name})
    
    file(GLOB ${subproject_name}_CMakeLists ${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt)
    
    file(GLOB ${subproject_name}_sources src/*.cpp)
    file(GLOB ${subproject_name}_headers src/*.h src/*.h.in src/*.hpp src/*.tpp
        ${${subproject_name}_top_include_dir}/*.h 
        ${${subproject_name}_top_include_dir}/*.h.in 
        ${${subproject_name}_top_include_dir}/*.hpp 
        ${${subproject_name}_top_include_dir}/*.tpp)
    file(GLOB ${subproject_name}_test_sources  tests/*.cpp)
    
    if(_qt)
        file(GLOB_RECURSE moc_${subproject_name}_sources moc_*.cpp *_automoc.cpp qrc_*.cpp)
        #remove generated moc files
        foreach(file_name ${moc_${subproject_name}_sources})
            list(REMOVE_ITEM ${subproject_name}_sources ${file_name})
        endforeach()
#---------------------------ADD QT UI FILES--------------------------------------------------------#
        file(GLOB ${subproject_name}_UI src/*.ui)
        
        if(BUILD_${_name})
        #this macro doesn't get defined unless QtWidgets is found
            qt5_wrap_ui(${subproject_name}_ui_headers ${${subproject_name}_ui})
        endif()
#---------------------------ADD QT RESOUCE FILES---------------------------------------------------#
        file(GLOB ${subproject_name}_resource_files *.qrc)
    
        if(BUILD_${_name})
            #this macro doesn't get defined unless QtWidgets is found
            qt5_add_resources(${subproject_name}_generated_resources ${${subproject_name}_resource_files})
        endif()
    endif()
    
    #..........................organize source/header files........................#
    source_group("Source Files" FILES ${${subproject_name}_sources})
    source_group("Header Files" FILES ${${subproject_name}_headers})
    
    if(_qt)
        source_group("Resource Files" FILES ${${subproject_name}_resource_files})
        source_group("UI Files" FILES ${${subproject_name}_ui})
        source_group("Generated Files" FILES ${${subproject_name}_generated_headers} ${${subproject_name}_ui_headers})
    endif()
    
    set(all_${subproject_name}_files
        ${${subproject_name}_CMakeLists}
    	${${subproject_name}_sources}
    	${${subproject_name}_headers}
    	${${subproject_name}_resource_files}
    	${${subproject_name}_ui}
    	${${subproject_name}_generated_headers}
    )
#---------------------------ADD TARGET-------------------------------------------------------------#
    if(${_subproject_type} STREQUAL "${app_type}")
        add_executable(${subproject_name} ${all_${subproject_name}_files})
    elseif(${_subproject_type} STREQUAL "${module_type}")
        add_library(${subproject_name} SHARED ${all_${subproject_name}_files})
    endif()
    
    #exclude from build if necessary
    if(NOT BUILD_${_name})
        set_target_properties(${module_name} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
        set (HAVE_${_name} FALSE PARENT_SCOPE)
    else()
        set (HAVE_${_name} TRUE PARENT_SCOPE)
    endif()
    
#---------------------------DEFINE INCLUDES--------------------------------------------------------#
    #local or project-specific includes
    target_include_directories(${subproject_name} PUBLIC include)
    
    #dependency includes
    foreach(depend ${_depends})
        if(DEFINED ${depend}_INCLUDE_DIRS)
            #by default, assume the includes are also required by projects that use this module
            #hence PUBLIC
            #TODO: add functionality to specify whether includes are PUBLIC/PRIVATE for each depend
            target_include_directories(${subproject_name} PUBLIC ${${depend}_INCLUDE_DIRS})
        endif()
    endforeach()
    
    get_target_property(${subproject_name}_includes ${subproject_name} INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "Includes: ${${subproject_name}_includes}")
        
#---------------------------LINK LIBRARIES --------------------------------------------------------#

    foreach(depend ${_depends})
        if(DEFINED ${depend}_LIBRARIES)
            #by default, assume the libraries are also required by projects that use this module
            #hence PUBLIC (may brake in case of static libs?)
            #TODO: add functionality to specify whether libraries are PUBLIC/PRIVATE for each depend
            target_link_libraries(${subproject_name} PUBLIC ${${depend}_LIBRARIES})
        endif()
    endforeach()
    
#---------------------------ADD PREPROCESSOR DEFINES-----------------------------------------------#
#TODO: add support for user-specified defines
    get_target_property(${subproject_name}_definitions ${subproject_name} COMPILE_DEFINITIONS)
    if(NOT "${${subproject_name}_definitions}")
        #clear out the "...-NOTFOUND" value
        set(${subproject_name}_definitions)
    endif()
    foreach(depend ${_depends})
        if(DEFINED ${depend}_DEFINITIONS)
            list(APPEND ${subproject_name}_definitions ${${depend}_DEFINITIONS})
        endif()
    endforeach()
    set_target_properties(${subproject_name} PROPERTIES COMPILE_DEFINITIONS "${${subproject_name}_definitions}")
endmacro()