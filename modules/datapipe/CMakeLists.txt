set(_module datapipe)

reco_add_subproject(${_module}
    DEPENDENCIES Qt5Core Qt5Widgets Qt5OpenGL OpenCV Boost
    MODULE QT)