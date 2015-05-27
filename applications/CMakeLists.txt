#Applications CMakeLists.txt
#=====================LARGE APPS===================================================================#
add_subdirectory("workbench")

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
    DEPENDENCIES LibDL HAL 420 Boost PCL
    LIGHTWEIGHT_APPLICATION)
    
reco_add_subproject(playback
    SOURCES playback.cpp
    DEPENDENCIES LibDL PCL Boost OpenCV Calibu
    LIGHTWEIGHT_APPLICATION)
    
# Kinfu library
#TODO: fix Alex's crude hack if necessary                                      
#add_library(kinfu SHARED kinfu.cpp)
#target_link_libraries(kinfu  ${PCL_LIBRARIES}
#                             ${Boost_LIBRARIES}
#                             ${OpenCV_LIBS}
#                                kinfu)
#TODO: fix Alex's crude hack if necessary
# Kinfu app
#add_executable( kinfuapp kinfu_app.cpp )
#target_link_libraries(kinfuapp  kinfu)