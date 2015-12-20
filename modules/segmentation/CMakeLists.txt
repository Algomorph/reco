set(_module segmentation)

reco_add_subproject(${_module}
    DEPENDENCIES OpenCV utils CUDA OpenMP
    MODULE)