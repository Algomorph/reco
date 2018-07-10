#set to 1,2 for debugging, 0 otherwise
set(verbosity 0)

reco_find_dependency(PythonLibs ${verbosity} QUIET LIBRARIES PYTHON_LIBRARIES INCLUDE_DIRS PYTHON_INCLUDE_DIRS)
reco_find_dependency(TinyXML ${verbosity} QUIET)
reco_find_dependency(OpenCV ${verbosity} QUIET
     COMPONENTS core highgui calib3d imgproc ximgproc xfeatures2d videoio video bgsegm optflow 
     cudastereo cudabgsegm cudalegacy cudaoptflow LIBRARIES OpenCV_LIBS)
reco_find_dependency(PCL ${verbosity} QUIET)
reco_find_dependency(Boost ${verbosity} QUIET COMPONENTS system filesystem python regex program_options) #has to come after PCL, BECAUSE.
reco_find_dependency(freenect2 ${verbosity} QUIET LIBRARIES freenect2_LIBRARY)
reco_find_dependency(SceneGraph ${verbosity} QUIET)
reco_find_dependency(Pangolin ${verbosity} QUIET)
reco_find_dependency(HAL ${verbosity} QUIET) 
reco_find_dependency(Protobuf ${verbosity} QUIET)
reco_find_dependency(Calibu ${verbosity} QUIET)
reco_find_dependency(Kangaroo ${verbosity} QUIET) 
reco_find_dependency(LibDL ${verbosity} QUIET)  
reco_find_dependency(VTK ${verbosity} QUIET)  
reco_find_dependency(x264 ${verbosity} QUIET LIBRARIES X264_LIBRARIES)
reco_find_dependency(FFMPEG ${verbosity} QUIET) 
reco_find_dependency(OpenMP ${verbosity} QUIET) 
reco_find_dependency(CUDA ${verbosity} QUIET)
reco_find_dependency(OpenGL ${verbosity} QUIET)
reco_find_dependency(Eigen3 ${verbosity} QUIET INCLUDE_DIRS EIGEN3_INCLUDE_DIRS)

if(CUDA_FOUND)
    #default to the CUDA version of OpenCL
    find_OpenCL_CUDA()
endif()
if(NOT OpenCL_FOUND)
    #Could not locate OpenCL in the nvidia CUDA package
    reco_find_dependency(OpenCL ${verbosity} QUIET)
endif()
reco_find_dependency(GLUT ${verbosity} QUIET)
reco_find_dependency(GLEW ${verbosity} QUIET)

#find python, numpy
include("DetectPython")
include("DetectQt") 

#temp fix for OpenKinect/libfreenect2 issue #217
#set(freenect2_INCLUDE_DIRS ${freenect2_INCLUDE_DIR} ${freenect2_INCLUDE_DIR}/libfreenect2/tinythread)