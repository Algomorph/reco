set(_module stereo_workbench)

reco_add_subproject(${_module}
    DEPENDENCIES datapipe utils stereo OpenCV HAL Calibu
    APPLICATION QT)