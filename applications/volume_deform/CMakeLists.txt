set(_module volume_deform)

reco_add_subproject(${_module}
    DEPENDENCIES utils Boost OpenCV voxels VTK Eigen3
    ADDITIONAL_INCLUDE_DIRS 
    APPLICATION)

include(${VTK_USE_FILE})