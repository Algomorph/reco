#Applications CMakeLists.txt

set (applications_built_by_default "stereo_workbench")

#=====================LARGE APPS===================================================================#
add_subdirectory("rgbd_workbench")
add_subdirectory("stereo_tuner")
add_subdirectory("stereo_workbench")

#=====================LIGHTWEIGHT APPS=============================================================#
reco_add_subproject(read_builtins 
    SOURCES read_builtins.cpp 
    DEPENDENCIES freenect2 OpenCV
    LIGHTWEIGHT_APPLICATION)

reco_add_subproject(capture 
    SOURCES capture.cpp
    DEPENDENCIES LibDL Boost Pangolin HAL
    LIGHTWEIGHT_APPLICATION)
    
reco_add_subproject(extract
    SOURCES extract.cpp
    DEPENDENCIES LibDL HAL Boost PCL
    LIGHTWEIGHT_APPLICATION)
    
reco_add_subproject(playback
    SOURCES playback.cpp
    DEPENDENCIES LibDL PCL Boost OpenCV Calibu HAL utils
    LIGHTWEIGHT_APPLICATION)
    
reco_add_subproject(stereo_rectify
    SOURCES stereo_rectify.cpp
    DEPENDENCIES OpenCV utils stereo
    LIGHTWEIGHT_APPLICATION)

reco_add_subproject(ogale_example
    SOURCES ogale_stereo_test.cpp
    DEPENDENCIES OpenCV utils stereo
    LIGHTWEIGHT_APPLICATION)

reco_add_subproject(subtract_background
    SOURCES subtract_background.cpp
    DEPENDENCIES OpenCV utils Boost
    ADDITIONAL_INCLUDE_DIRS /usr/local/cuda-7.5/targets/x86_64-linux/include
    LIGHTWEIGHT_APPLICATION)
    
reco_add_subproject(optical_flow_contours
    SOURCES optical_flow_contours.cpp
    DEPENDENCIES OpenCV utils Boost
    ADDITIONAL_INCLUDE_DIRS 
    LIGHTWEIGHT_APPLICATION)
    
reco_add_subproject(gpu_quickshift
    SOURCES gpu_quickshift.cpp
    DEPENDENCIES utils Boost segmentation CUDA OpenCV
    ADDITIONAL_INCLUDE_DIRS 
    LIGHTWEIGHT_APPLICATION)