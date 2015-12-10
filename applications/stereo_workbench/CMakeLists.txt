set(_module stereo_workbench)

reco_add_subproject(${_module}
    DEPENDENCIES utils OpenCV Boost OpenMP OpenCL OpenGL GLEW GLUT     
    ADDITIONAL_INCLUDE_DIRS 
    APPLICATION)