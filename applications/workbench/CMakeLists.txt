set(_module workbench)

reco_add_subproject(${_module}
    DEPENDENCIES datapipe utils OpenCV HAL VTK PCL
    APPLICATION QT)