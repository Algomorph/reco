set(_module stereo_workbench)

reco_add_subproject(${_module}
    DEPENDENCIES utils stereo OpenCV Boost OpenMP OpenCL OpenGL GLEW GLUT     
    ADDITIONAL_INCLUDE_DIRS /usr/local/cuda-7.5/targets/x86_64-linux/include/
    APPLICATION)