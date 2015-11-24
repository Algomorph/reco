set(_module stereo)

reco_add_subproject(${_module}
    DEPENDENCIES OpenCV utils Calibu
    MODULE)