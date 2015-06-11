set(_module python)

reco_add_subproject(${_module} 
    DEPENDENCIES PythonLibs OpenCV Boost 
    REQUIREMENTS PYTHON_USE_NUMPY  
    MODULE )

#---------------------------   INSTALLATION    -----------------------------------------------------
#-get proper extension for python binary shared object on this platform
if(BUILD_${_module})
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import distutils.sysconfig; print distutils.sysconfig.get_config_var('SO')"
                    RESULT_VARIABLE PYTHON_RECO_PY_PROCESS
                    OUTPUT_VARIABLE RECO_PY_SUFFIX
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
    
    set_target_properties(${subproject_name} PROPERTIES
                          PREFIX ""
                          OUTPUT_NAME reco
                          SUFFIX ${RECO_PY_SUFFIX})
    
    if(MSVC AND NOT PYTHON_DEBUG_LIBRARIES)
      set(PYTHON_INSTALL_CONFIGURATIONS CONFIGURATIONS Release)
    else()
      set(PYTHON_INSTALL_CONFIGURATIONS "")
    endif()
    
    if(WIN32)
      set(PYTHON_INSTALL_ARCHIVE "")
    else()
      set(PYTHON_INSTALL_ARCHIVE ARCHIVE DESTINATION ${PYTHON_PACKAGES_PATH} COMPONENT python)
    endif()
    
    install(TARGETS ${subproject_name}
          ${PYTHON_INSTALL_CONFIGURATIONS}
          RUNTIME DESTINATION ${PYTHON_PACKAGES_PATH} COMPONENT python
          LIBRARY DESTINATION ${PYTHON_PACKAGES_PATH} COMPONENT python
          ${PYTHON_INSTALL_ARCHIVE}
     )
 endif()