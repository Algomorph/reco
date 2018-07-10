set(_module math)

reco_add_subproject(${_module} 
    DEPENDENCIES utils Eigen3
    MODULE)