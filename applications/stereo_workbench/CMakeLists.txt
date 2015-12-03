set(_module stereo_workbench)

reco_add_subproject(${_module}
    DEPENDENCIES utils stereo OpenCV Boost
    APPLICATION)