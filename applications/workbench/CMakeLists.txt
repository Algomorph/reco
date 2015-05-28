set(_module workbench)

reco_add_subproject(${_module}
    DEPENDENCIES datapipe OpenCV 
    APPLICATION QT)