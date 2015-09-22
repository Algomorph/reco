set(_module stereo_workbench)

reco_add_subproject(${_module}
    DEPENDENCIES datapipe utils OpenCV HAL halext VTK PCL
    APPLICATION QT)