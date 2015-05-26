set(_module workbench)

reco_add_subproject(${_module}
    DEPENDENCIES Qt5Core Qt5Widgets Qt5OpenGL OpenCV datapipe
    APPLICATION QT)