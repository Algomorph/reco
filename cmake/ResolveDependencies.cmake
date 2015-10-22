reco_find_dependency(PythonLibs QUIET LIBRARIES PYTHON_LIBRARIES INCLUDE_DIRS PYTHON_INCLUDE_DIRS)
reco_find_dependency(TinyXML QUIET)
reco_find_dependency(OpenCV QUIET COMPONENTS core highgui calib3d imgproc LIBRARIES OpenCV_LIBS)
reco_find_dependency(PCL QUIET)
reco_find_dependency(Boost QUIET COMPONENTS system filesystem python regex) #has to come after PCL, because. 
reco_find_dependency(freenect2 QUIET LIBRARIES freenect2_LIBRARY)
reco_find_dependency(SceneGraph QUIET)
reco_find_dependency(Pangolin QUIET)
reco_find_dependency(HAL QUIET)
reco_find_dependency(Protobuf QUIET)
reco_find_dependency(Calibu QUIET)
reco_find_dependency(Kangaroo QUIET)
reco_find_dependency(LibDL QUIET)
reco_find_dependency(VTK QUIET)

#find python, numpy
include("DetectPython")
include("DetectQt")

#temp fix for OpenKinect/libfreenect2 issue #217
set(freenect2_INCLUDE_DIRS ${freenect2_INCLUDE_DIR} ${freenect2_INCLUDE_DIR}/libfreenect2/tinythread)