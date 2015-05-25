include("RecoDependencies")
include("RecoModule")

macro(reco_check_environment_variables)
  foreach(_var ${ARGN})
    if(NOT DEFINED ${_var} AND DEFINED ENV{${_var}})
      set(__value "$ENV{${_var}}")
      file(TO_CMAKE_PATH "${__value}" __value) # Assume that we receive paths
      set(${_var} "${__value}")
      message(STATUS "Update variable ${_var} from environment: ${${_var}}")
    endif()
  endforeach()
endmacro()

macro(puts _text)
    message(STATUS ${_text})
endmacro()