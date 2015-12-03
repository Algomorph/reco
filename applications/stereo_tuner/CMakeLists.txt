set(_module stereo_tuner)

reco_add_subproject(${_module}
    DEPENDENCIES datapipe utils stereo OpenCV HAL Calibu
    APPLICATION QT)