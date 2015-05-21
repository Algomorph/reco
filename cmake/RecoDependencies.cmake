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
    set(keywords COMPONENTS REQUIRED QUIET LIBRARIES INCLUDE_DIRS DEFINITIONS)
    
    set(_components)
    set(append_components 0)
    set(set_libraries 0)
    set(set_includes 0)
    set(set_defines 0)
    
    set(_required)
    set(_quiet)
    set(_libraries 0)
    set(_includes 0)
    set(_defines 0)
    
    foreach(arg ${ARGN})
        #check for keywords
        list(FIND keywords ${arg} index)
        if(index EQUAL -1)
            if(append_components)
                #append components one by one until you hit a keyword
                list(APPEND _components ${arg})
            elseif(set_libraries)
                #set the libraries var name
                set(_libraries ${arg})
                set(set_libraries 0)
            elseif(set_includes)
                #set the includes var name
                set(_includes ${arg})
                set(set_includes 0)
            elseif(set_defines)
                #set the defines var name
                set(_defines ${arg})
                set(set_defines 0)
            endif()
        else()
            set(append_components 0)
            #check which keyword
            if("${arg}" STREQUAL "COMPONENTS")
               set(append_components 1)
            elseif("${arg}" STREQUAL "REQUIRED") 
               set(_required "REQUIRED")
            elseif("${arg}" STREQUAL "QUIET")
               set(_quiet "QUIET")
            elseif("${arg}" STREQUAL "LIBRARIES")
               set(set_libraries 1)
            elseif("${arg}" STREQUAL "INCLUDE_DIRS")
               set(set_includes 1)
            elseif("${arg}" STREQUAL "DEFINITIONS")
               set(set_defines 1)
            endif()
        endif()
    endforeach()
    
    
    find_package(${_name} ${_required} ${_quiet} COMPONENTS ${_components})
    
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
