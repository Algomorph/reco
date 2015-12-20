#add dependency include dirs to subproject
macro(reco_add_includes_to_subproject _target_name)
    set(_depends ${ARGN})
    #dependency includes
    foreach(depend ${_depends})
        if(DEFINED ${depend}_INCLUDE_DIRS)
            #by default, assume the includes are also required by projects that use this module
            #hence PUBLIC
            #TODO: add functionality to specify whether includes are PUBLIC/PRIVATE for each depend
            target_include_directories(${_target_name} PUBLIC ${${depend}_INCLUDE_DIRS})
        endif()
    endforeach()
endmacro()

#link dependency libraries to subproject
macro(reco_link_libraries_to_subproject _target_name verbose link_plain)
    set(_depend_libs)
    set(_depends ${ARGN})
    #dependency libraries
    foreach(depend ${_depends})
        if(DEFINED ${depend}_LIBRARIES)
            #by default, assume the libraries are also required by projects that use this module
            #hence PUBLIC (may brake in case of static libs?)
            #TODO: add functionality to specify whether libraries are PUBLIC/PRIVATE for each depend
            if(${link_plain})
                target_link_libraries(${_target_name} ${${depend}_LIBRARIES})
            else()
                target_link_libraries(${_target_name} PUBLIC ${${depend}_LIBRARIES})
            endif()
            list(APPEND _depend_libs ${${depend}_LIBRARIES})
        endif()
    endforeach()
    if(${verbose} EQUAL 1)
        get_target_property(${_target_name}_libs ${_target_name} LINK_INTERFACE_LIBRARIES)
        message(STATUS "Libraries linked to subproject '${_target_name}': ${${_target_name}_libs}")
    endif() 
endmacro()

#add dependency preprocessor definitions and compile flags to subproject
macro(reco_add_depends_to_subproject _target_name verbose cuda_subproject)
    set(_depends ${ARGN})
    if(NOT "${${_target_name}_definitions}")
        #clear out the "...-NOTFOUND" value
        set(${_target_name}_definitions)
    endif()
    foreach(depend ${_depends})
        if(DEFINED ${depend}_DEFINITIONS)
            string(REPLACE "-D" "" filtered_defs ${${depend}_DEFINITIONS})
            list(APPEND ${_target_name}_definitions ${filtered_defs})
        endif()
    endforeach()
    set_target_properties(${_target_name} PROPERTIES COMPILE_DEFINITIONS "${${_target_name}_definitions}")
    if(${verbose} EQUAL 1)
        get_target_property(${_target_name}_definitions ${_target_name} COMPILE_DEFINITIONS)
        message(STATUS "${_target_name} defs: ${${_target_name}_definitions}")
    endif()

endmacro()

macro(reco_add_depend_flags verbose cuda_subproject)
    set(_depends ${ARGN})
    if(${cuda_subproject})
        get_property(folder_compile_defs DIRECTORY PROPERTY COMPILE_DEFINITIONS)
        foreach(def ${folder_compile_defs})
            if(NOT ${def} STREQUAL "_DEBUG") 
               remove_definitions("-D${def}")
            endif() 
        endforeach() 
    endif()
    foreach(depend ${_depends})
        if(DEFINED ${depend}_CXX_FLAGS)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${${depend}_CXX_FLAGS}")
        endif()
        if(DEFINED ${depend}_SHARED_LINKER_FLAGS )
            set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${${depend}_SHARED_LINKER_FLAGS }")
        endif()
        if(DEFINED ${depend}_MODULE_LINKER_FLAGS )
            set(CMAKE_SHARED_MODULE_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${${depend}_MODULE_LINKER_FLAGS }")
        endif() 
    endforeach()
endmacro()

#report failure on repeatedly setting a parameter that can only be set once
#TODO: generify, adding a parameter_name parameter and using that instead of "module type"
macro(reco_check_repeated_subproject_type  _subproject_type _arg)
    if(_subproject_type)
        message(FATAL_ERROR "Trying to set module type repeatedly. Already have: '${_subproject_type}'. Now parsed: '${arg}'.")
    endif()
endmacro()

####################################################################################################
#  CMake macro that abstracts away adding modules(library subprojects) and applications(executable 
#  subprojects) in CMake, i.e. turns that into a one-liner.
#
#  Syntax: 
#  reco_add_subproject(<subproject_name>  
#    MODULE [QT] | APPLICATION [QT] | LIGHTWEIGHT_APPLICATION
#    [[DEPENDENCIES] [names of packages or modules that this subproject depends on...]]
#    [[SOURCES] [project sources...]]
#    [TEST])
#
#  If the QT flag is passed in, this will let the script know that the module uses the qmake moc 
#  compiler (qt resource files, ui files, and custom widgets are supported).
#  If the TEST flag (currently unsupported) is passed in, the macro will generate an additional test 
#  target using GTest.
#  For the MODULE and APPLICATION types, the SOURCES are optional, and are used to specify additional
#  sources which are not int <current dir>/src directory. For the LIGHTWEIGHT_APPLICATION type, they
#  are mandatory, as it will not look for any files within an "src" folder.
#
####################################################################################################
macro(reco_add_subproject _name)
    set(subproject_name ${global_project_name}_${_name})
    
    #adjust verbocity level here to 1, 2 for debugging
    set(verbose 0)

#----------------------------------PARSE ARGUMENTS-------------------------------------------------#

    #subproject types    
    set(module_type "MODULE")
    set(app_type "APPLICATION")
    set(lightweight_app_type "LIGHTWEIGHT_APPLICATION")
    
    set(subproject_types ${module_type} ${app_type} ${lightweight_app_type})
    
    #other keywords
    set(depends_keyword "DEPENDENCIES")
    set(qt_keyword "QT")
    set(sources_keyword "SOURCES")
    set(reqs_keyword "REQUIREMENTS")
    set(additional_includes_keyword "ADDITIONAL_INCLUDE_DIRS")

    set(keywords ${depends_keyword} ${qt_keyword} ${sources_keyword} ${reqs_keyword} ${additional_includes_keyword})
    list(APPEND keywords ${subproject_types})
    
    set(append_depends 0)
    set(append_sources 0)
    set(append_reqs 0)
    set(append_incls 0)
    set(_sources)
    set(_depends)
    set(_incls)
    set(_reqs)
    set(_qt 0)
    set(_subproject_type 0)
    

    foreach(arg ${ARGN})
        #check if it's a keyword
        list(FIND keywords ${arg} index)
        if(index EQUAL -1)
            #append to lists
            if(append_depends)
                list(APPEND _depends ${arg})
            elseif(append_sources)
                list(APPEND _sources ${arg})
            elseif(append_reqs)
                list(APPEND _reqs ${arg})
            elseif(append_incls)
                list(APPEND _incls ${arg})
            endif()
        else()
            set(append_depends 0)
            set(append_sources 0)
            set(append_reqs 0)
            set(append_incls 0)
            #check which keyword
            if("${arg}" STREQUAL "${depends_keyword}")
                set(append_depends 1)
            elseif("${arg}" STREQUAL "${sources_keyword}")
                set(append_sources 1)
            elseif("${arg}" STREQUAL "${reqs_keyword}")
                set(append_reqs 1)
            elseif("${arg}" STREQUAL "${additional_includes_keyword}")
                set(append_incls 1)
            elseif("${arg}" STREQUAL "${qt_keyword}")
                set(_qt 1)
            elseif("${arg}" STREQUAL "${module_type}")
                reco_check_repeated_subproject_type(${_subproject_type} ${arg})
                set(_subproject_type ${module_type})
            elseif("${arg}" STREQUAL "${app_type}")
                reco_check_repeated_subproject_type(${_subproject_type} ${arg})
                set(_subproject_type ${app_type})
            elseif("${arg}" STREQUAL "${lightweight_app_type}")
                reco_check_repeated_subproject_type(${_subproject_type} ${arg})
                set(_subproject_type ${lightweight_app_type})
            endif()
        endif()
    endforeach()
    
    #consistency/argument checks
    if(NOT _subproject_type)
        message(FATAL_ERROR "Type of module not set. Expecting one of [${subproject_types}] as argument.")
    endif()
    
    if(${_subproject_type} STREQUAL ${lightweight_app_type} AND _qt)
        message(FATAL_ERROR "LIGHTWEIGHT_APPLICATION and QT arguments to the reco_add_subproject are incompatible.")
    endif()
#---------------------------- PROJECT NAME --------------------------------------------------------#
    if(NOT ${_subproject_type} STREQUAL ${lightweight_app_type})
        project(${subproject_name})
    endif()    
#-----------------------------CHECK DEPENDENCIES AND REQUIREMENTS----------------------------------#

 #scal for cuda in dependencies, since that changes the command to add target
    set(cuda_subproject FALSE)
 #depends
    set(_have_depends TRUE)
    set(_unmet_depends)
    foreach(dep ${_depends}) 
        if(NOT HAVE_${dep})
            set(_have_depends FALSE)
            list(APPEND _unmet_depends ${dep})
        elseif(${dep} STREQUAL CUDA)
            set(cuda_subproject TRUE)
        endif()
    endforeach()
 #reqs
    set(_reqs_satisfied TRUE)
    set(_unmet_reqs)
    foreach(req ${_reqs})
        if(NOT ${${req}})
            set(_reqs_satisfied FALSE)
            list(APPEND _unmet_reqs ${req})
        endif()
    endforeach()
    
    if(${_reqs_satisfied} AND ${_have_depends})
        set(_can_build TRUE)
    else()
        set(_can_build FALSE)
    endif()
    
    if(NOT DEFINED BUILD_${_name})
        # also check if build is enabled by default
        set(_built_by_default ${modules_built_by_default} ${applications_built_by_default})
        list(FIND _built_by_default ${_name} _def_ix)
        if(_def_ix GREATER -1)
            SET(BUILD_${_name} ${_can_build} CACHE BOOL "Build the '${_name}' module.")
            if(_def_ix GREATER -1)
                if(NOT ${_have_depends})
                    message(WARNING "Cannot build module '${_name}', the following dependencies are not met: ${_unmet_depends}. Setting BUILD_${_name} to FALSE.")
                endif()
                if(NOT ${_reqs_satisfied})
                    message(WARNING "Cannot build module '${_name}', the following requirements are not met: ${_unmet_reqs}. Setting BUILD_${_name} to FALSE.")
                endif()
            endif()
        else()
            SET(BUILD_${_name} FALSE CACHE BOOL "Build the '${_name}' module.")
        endif()
    else()
        if(BUILD_${_name})
            if(NOT ${_have_depends})
                message(FATAL_ERROR "Cannot build module '${_name}', the following dependencies are not met: ${_unmet_depends}. Setting BUILD_${_name} to FALSE.")
            endif()
            if(NOT ${_reqs_satisfied})
                message(FATAL_ERROR "Cannot build module '${_name}', the following requirements are not met: ${_unmet_reqs}")
            endif()
        endif()
    endif()
   
#--------------------------- AUTOMOC FOR QT -------------------------------------------------------#    
    if(_qt)
        set(CMAKE_AUTOMOC ON)
        set(CMAKE_INCLUDE_CURRENT_DIR ON)
    endif()

#---------------------------DEFINE SOURCE FILES----------------------------------------------------#
    SET(${subproject_name}_top_include_dir include/${global_project_name}/${_name})
    
    file(GLOB ${subproject_name}_CMakeLists ${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt)
    
    set(${subproject_name}_sources)
    if(NOT ${_subproject_type} STREQUAL "${lightweight_app_type}")
        #search for all sources in the current source dir
        file(GLOB_RECURSE ${subproject_name}_sources src/ *.cpp *.cu)
        file(GLOB_RECURSE ${subproject_name}_headers src/ *.h *.h.in *.hpp *.tpp)
        file(GLOB_RECURSE ${subproject_name}_headers ${${subproject_name}_top_include_dir}/ *.h *.h.in *.hpp *.tpp)
        file(GLOB_RECURSE ${subproject_name}_test_sources tests/ *.cpp)
    endif()
    
    #append sources specified in the arguments
    list(APPEND ${subproject_name}_sources ${_sources})
    
    if(_qt)
        file(GLOB_RECURSE moc_${subproject_name}_sources moc_*.cpp *_automoc.cpp qrc_*.cpp)
        #remove generated moc files
        foreach(file_name ${moc_${subproject_name}_sources})
            list(REMOVE_ITEM ${subproject_name}_sources ${file_name})
        endforeach()
#---------------------------ADD QT UI FILES--------------------------------------------------------#
        file(GLOB ${subproject_name}_ui qt/*.ui)
        
        if(BUILD_${_name})
        #this macro doesn't get defined unless QtWidgets is found
            qt5_wrap_ui(${subproject_name}_ui_headers ${${subproject_name}_ui})
        endif()
#---------------------------ADD QT RESOUCE FILES---------------------------------------------------#
        file(GLOB ${subproject_name}_resource_files qt/*.qrc)
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
        source_group("Generated Files" FILES 
            ${${subproject_name}_ui_headers}
            ${${subproject_name}_generated_resources}
        )
    endif()
    
    set(all_${subproject_name}_files
        ${${subproject_name}_CMakeLists}
    	${${subproject_name}_sources}
    	${${subproject_name}_headers}
    	${${subproject_name}_resource_files}
    	${${subproject_name}_ui}
    	${${subproject_name}_ui_headers}
        ${${subproject_name}_generated_resources}
    )   
    
    #${subproject_name}_target_files is an intermediate list in case not all project files
    #need to be added to the target in the future
    set(${subproject_name}_target_files ${all_${subproject_name}_files})

    if(verbose EQUAL 1)
        message(STATUS "Files for subproject ${subproject_name}: ${all_${subproject_name}_files}")
    elseif(verbose EQUAL 2)
        message(STATUS "Files for subproject ${subproject_name}:")
        message(STATUS "     CMake: ${${subproject_name}_CMakeLists}")
        message(STATUS "     Source Files: ${${subproject_name}_sources}")
        message(STATUS "     Header Files: ${${subproject_name}_headers}")
        message(STATUS "     Resource Files: ${${subproject_name}_resource_files}")
        message(STATUS "     Ui Files: ${${subproject_name}_ui}")
        message(STATUS "     Ui Generated Headers: ${${subproject_name}_ui_headers}")
        message(STATUS "     Resource Generated Headers: ${${subproject_name}_generated_resources}")
    endif()
#---------------------------HANDLE EXTRA COMPILE AND LINKER FLAGS----------------------------------#
    reco_add_depend_flags(${verbose} ${cuda_subproject} ${_depends})
    
#---------------------------ADD TARGET-------------------------------------------------------------#
    if(${_subproject_type} STREQUAL "${app_type}" OR ${_subproject_type} STREQUAL "${lightweight_app_type}")
        #don't use the subproject name, but rather the briefer name suffix for the executables
        #this will make command-line invocation involve less typing
        set(_target_name ${_name})
        if(cuda_subproject)
            cuda_add_executable(${_target_name} ${${subproject_name}_target_files})
        else()
            add_executable(${_target_name} ${${subproject_name}_target_files})
        endif()
    elseif(${_subproject_type} STREQUAL "${module_type}")
        set(_target_name ${subproject_name})
        if(cuda_subproject)
            cuda_add_library(${_target_name} SHARED ${${subproject_name}_target_files})
        else()
            add_library(${_target_name} SHARED ${${subproject_name}_target_files})
        endif()
    endif()
       
    #exclude from build if necessary
    if(NOT BUILD_${_name})
        set_target_properties(${_target_name} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
        #this syntax is required to propagate these to the upper level
        set (HAVE_${_name} FALSE CACHE INTERNAL "have flag" FORCE)
    else()
        set (HAVE_${_name} TRUE CACHE INTERNAL "have flag" FORCE)
    endif()
    
    
    
#---------------------------DEFINE INCLUDES--------------------------------------------------------#
    #local or project-specific *public* includes
    target_include_directories(${_target_name} PUBLIC include)
    
    reco_add_includes_to_subproject(${_target_name} ${_depends})
    
    if(_incls)
        target_include_directories(${_target_name} PUBLIC ${_incls})
    endif()
    
    if(verbose EQUAL 2)
    get_target_property(${_target_name}_INCLUDE_DIRS ${_target_name} INCLUDE_DIRECTORIES)
    message(STATUS "Include directories: ${${_target_name}_INCLUDE_DIRS}")
    endif()
#---------------------------LINK LIBRARIES --------------------------------------------------------#

    reco_link_libraries_to_subproject(${_target_name} ${verbose} ${cuda_subproject} ${_depends})
    
    #make a list of used libraries, for reuse in applications
    if(${_subproject_type} STREQUAL ${module_type} AND BUILD_${_name})
        #TODO: not sure if dependency libs are required 
        #(shouldn't CMake add those automatically when the target subproject is linked via target_link_libraries?)
        #SET(${_name}_LIBRARIES ${_depend_libs} ${subproject_name} CACHE INTERNAL "module library define" FORCE)
        SET(${_name}_LIBRARIES ${subproject_name} CACHE INTERNAL "module library define" FORCE)
    endif()
    
#---------------------------ADD PREPROCESSOR DEFINES-----------------------------------------------#
#TODO: add support for user-specified defines
    reco_add_depends_to_subproject(${_target_name} ${verbose} ${cuda_subproject} ${_depends})
    
endmacro()