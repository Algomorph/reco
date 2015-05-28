if(WIN32 AND (DEFINED ENV{QT_DIR}) AND IS_DIRECTORY "$ENV{QT_DIR}/lib/cmake")
    #if we're on windows, see if QT_DIR environment variable is set.
	SET(QT_CMAKE_HINTS "$ENV{QT_DIR}/lib/cmake") 
endif()

set(QT_COMPONENTS Qt5Core Qt5Widgets Qt5OpenGL)

reco_find_dependency(Qt5Core QUIET HINTS ${QT_CMAKE_HINTS})
reco_find_dependency(Qt5Widgets QUIET HINTS ${QT_CMAKE_HINTS})
reco_find_dependency(Qt5OpenGL QUIET HINTS ${QT_CMAKE_HINTS})

set(QT_TARGETS
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${Qt5OpenGL_LIBRARIES}
)
#choose between debug and release qt libraries
if(NOT MSVC)#not necessary for MSVC - it uses both debug & release version
    if(${CMAKE_BUILD_TYPE} STREQUAL "Debug" OR ${CMAKE_BUILD_TYPE} STREQUAL "RelWithDebInfo")
        set(QT_CONFIG "DEBUG")
    else()
        set(QT_CONFIG "RELEASE")
    endif()
    foreach(QT_TARGET ${QT_TARGETS})
        set_target_properties(${QT_TARGET} PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE ${QT_CONFIG})
    endforeach()
endif()