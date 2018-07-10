set(_module voxels)

reco_add_subproject(${_module} 
    DEPENDENCIES Boost OpenCV utils math Eigen3
    MODULE)