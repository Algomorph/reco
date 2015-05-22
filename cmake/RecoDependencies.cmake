####################################################################################################
#  CMake macro that abstracts away some hassle behind the usual CMake's find_package.
#
#  Syntax: 
#  reco_find_dependency(<package> [QUIET] [REQUIRED] 
#    [[COMPONENTS] [components...]] [LIBRARIES [libraries_var_name]] 
#    [INCLUDE_DIRS [include_dirs_var_name]] [DEFINITIONS [definitions_var_name]])
#
#  Macro assumes that the call to find_package(<package> ...) sets either <package>_FOUND or 
#  <capitalized_package>_FOUND variables to TRUE upon success. In such case, it sets the 
#  HAVE_<package> variable to TRUE. 
#
#  Many find_package scripts do not follow the convention of <package>_LIBRARIES, 
#  <package>_INCLUDE_DIRS, and <package>_DEFINITIONS. In such cases, the libraries_var_name,
#  include_dirs_var_name, and/or definitions_var_name need to be set to what the find_package script
#  for that package uses, such that the macro will automatically assign the values of these to the 
#  given conventional variable names, thus normalizing usage of the package in later CMake code.
#
#  Not all find_package arguments are currently supported by the reco_find_dependency macro. If 
#  arguments that are not yet supported need to be passed, we will augment the functionality of the 
#  script to accommodate this.
####################################################################################################

macro(reco_find_dependency _name)
    #parse arguments
    set(comps_keyword "COMPONENTS")
    set(libs_keyword "LIBRARIES")
    set(includes_keyword "INCLUDE_DIRS")
    set(defs_keyword "DEFINITIONS")
    set(req_keyword "REQUIRED")
    set(quiet_keyword "QUIET")
    set(hints_keyword "HINTS")
    
    set(keywords ${comps_keyword} ${libs_keyword} ${includes_keyword} 
        ${defs_keyword} ${req_keyword} ${quiet_keyword})
    
    set(_components)
    set(_hints)
    
    set(_comps_directive)
    set(_hints_directive)
    
    set(_append_components FALSE)
    set(_append_hints FALSE)
    
    set(_set_libraries FALSE)
    set(_set_includes FALSE)
    set(_set_defines FALSE)
    
    set(_required)
    set(_quiet)
    set(_libraries FALSE)
    set(_includes FALSE)
    set(_defines FALSE)
    
    foreach(arg ${ARGN})
        #check for keywords
        list(FIND keywords ${arg} index)
        if(index EQUAL -1)
            if(_append_components)
                #append components one by one until you hit a keyword
                list(APPEND _components ${arg})
            elseif(_append_hints)
                list(APPEND _hints ${arg})
            elseif(_set_libraries)
                #set the libraries var name
                set(_libraries ${arg})
                set(_set_libraries FALSE)
            elseif(_set_includes)
                #set the includes var name
                set(_includes ${arg})
                set(_set_includes FALSE)
            elseif(_set_defines)
                #set the defines var name
                set(_defines ${arg})
                set(_set_defines FALSE)
            endif()
        else()
            set(_append_components 0)
            set(_append_hints FALSE)
            #check which keyword
            if("${arg}" STREQUAL ${comps_keyword})
                set(_comps_directive ${comps_keyword})
                set(_append_components TRUE)
            elseif("${arg}" STREQUAL ${hints_keyword}) 
                set(_hints_directive ${hints_keyword})
                set(_append_hints TRUE)
            elseif("${arg}" STREQUAL ${req_keyword}) 
                set(_required "REQUIRED")
            elseif("${arg}" STREQUAL ${quiet_keyword})
                set(_quiet "QUIET")
            elseif("${arg}" STREQUAL ${libs_keyword})
                set(_set_libraries TRUE)
            elseif("${arg}" STREQUAL ${includes_keyword})
                set(_set_includes TRUE)
            elseif("${arg}" STREQUAL ${defs_keyword})
                set(_set_defines TRUE)
            endif()
        endif()
    endforeach()
    
    find_package(${_name} ${_required} ${_quiet} ${_comps_directive} ${_components} ${_hints_directive} ${_hints})
    
    string(TOUPPER ${_name} _name_upper)

    if(${_name}_FOUND OR ${_name_upper}_FOUND)

        set(HAVE_${_name} TRUE)
        #standardise libraries, includes, defines variable names
        if(${_libraries})
            set(${_name}_LIBRARIES  ${${_libraries}})
        endif()
        if(${_includes})
            set(${_name}_INCLUDE_DIRS  ${${_includes}})
        endif()
        if(${_defines})
            set(${_name}_DEFINITIONS  ${${_defines}})
        endif()
    else()
        set(HAVE_${_name} FALSE)
    endif()

endmacro()
