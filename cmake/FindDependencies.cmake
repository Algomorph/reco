reco_find_dependency(PythonLibs QUIET LIBRARIES PYTHON_LIBRARIES INCLUDE_DIRS PYTHON_INCLUDE_DIRS)
reco_find_dependency(OpenCV QUIET COMPONENTS core highgui calib3d imgproc LIBRARIES OpenCV_LIBS)
reco_find_dependency(Boost QUIET COMPONENTS regex system filesystem python)
reco_find_dependency(PCL QUIET)