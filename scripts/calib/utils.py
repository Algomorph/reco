'''
Created on Nov 23, 2015

@author: algomorph
'''

import numpy as np

import cv2
import time
import calib.io as io

def calibrate_solo(objpoints,imgpoints,frame_dims, K, d, flags, criteria, ix_cam):
    start = time.time()
    err, K, d, = cv2.calibrateCamera(objpoints, imgpoints, 
                                     frame_dims, K, d, flags=flags,criteria = criteria)[0:3]
    end = time.time()
    print ("Camera {0:d} calibration.\n   Time (s): {1:f}\n"+
           "K:\n{2:s}\nd:\n{3:s}\nError: {4:f}".format(ix_cam,end - start,str(K),str(d),err))
    return err, K, d

def generate_preview(K1, d1, K2, d2, R, T, test_im_left, test_im_right):
    im_size = test_im_left.shape
    new_size = (int(im_size[1]*1.5),int(im_size[0]*1.5))
    R1, R2, P1, P2 = cv2.stereoRectify(cameraMatrix1=K1, 
                        distCoeffs1=d1, 
                        cameraMatrix2=K2, 
                        distCoeffs2=d2, 
                        imageSize=im_size, 
                        R=R,T=T, flags=cv2.CALIB_ZERO_DISPARITY, 
                        newImageSize=new_size)[0:4]
    map1x, map1y = cv2.initUndistortRectifyMap(K1, d1, R1, P1, new_size, cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(K2, d2, R2, P2, new_size, cv2.CV_32FC1)
    
    rect_left = cv2.remap(test_im_left, map1x, map1y, cv2.INTER_LINEAR)
    rect_right = cv2.remap(test_im_right, map2x, map2y, cv2.INTER_LINEAR)
    return rect_left, rect_right

def calibrate(limgpoints,rimgpoints,objpoints,
              frame_dims,
              use_fisheye = False,
              use_8 = True,
              use_tangential = False,
              precalibrate_solo = True, 
              max_iters = 30,
              path_to_calib_file = None):
    flags = 0    
    if path_to_calib_file != None:
        prev_error, K1, d1, K2, d2, R, T, prev_dims = io.load_opencv_calibration(path_to_calib_file)  # @UnusedVariable
        if(frame_dims != prev_dims):
            raise ValueError("Frame dimensions in specified calibration file (" + 
                             path_to_calib_file + " don't correspond to video files.")
        flags += cv2.CALIB_USE_INTRINSIC_GUESS
    else:
        K1 = np.eye(3,dtype=np.float64)
        K2 = np.eye(3,dtype=np.float64)
    
    criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, max_iters, 2.2204460492503131e-16)
    
    
    if use_fisheye:
        E = None
        F = None
        if path_to_calib_file == None:  
            d1 = np.zeros(4,np.float64)
            d2 = np.zeros(4,np.float64)
        obp2 = []
        for pointset in objpoints:
            obp2.append(np.transpose(pointset,(1,0,2)).astype(np.float64))
        
        lpts2 = []
        for pointset in limgpoints:
            lpts2.append(np.transpose(pointset,(1,0,2)).astype(np.float64))
        
        rpts2 = []
        for pointset in rimgpoints:
            rpts2.append(np.transpose(pointset,(1,0,2)).astype(np.float64))
        #err, K1, d1, tvecs, rvecs = cv2.fisheye.calibrate(objectPoints = obp2, imagePoints = lpts2, 
        #                                                  image_size = frame_dims, K=K1, D=d1)
        #TODO: try this with transposed matrices
        error, K1, d1, K2, d2, R, T \
            = cv2.fisheye.stereoCalibrate(obp2, lpts2, rpts2, 
                                          K1, d1, K2, d2, 
                                          imageSize=frame_dims,
                                          flags=flags,
                                          criteria=criteria)
    else:
        if path_to_calib_file == None:
            d1 = np.zeros(8,np.float64)
            d2 = np.zeros(8,np.float64)
        if not use_tangential:
            flags += cv2.CALIB_ZERO_TANGENT_DIST
        if use_8:
            flags += cv2.CALIB_RATIONAL_MODEL
        
        if(precalibrate_solo):
            err1, K1, d1 = calibrate_solo(objpoints, limgpoints, frame_dims, K1, d1, flags, criteria, 1)  # @UnusedVariable
            err2, K2, d2 = calibrate_solo(objpoints, rimgpoints, frame_dims, K2, d2, flags, criteria, 2)  # @UnusedVariable

            flags += cv2.CALIB_FIX_INTRINSIC
            
            if(int(cv2.__version__ [0]) == 2):
                error, K1, d1, K2, d2, R, T, E, F \
                    = cv2.stereoCalibrate(objpoints,limgpoints,rimgpoints,  
                                          imageSize = frame_dims,
                                          cameraMatrix1 = K1, distCoeffs1 = d1, 
                                          cameraMatrix2 = K2, distCoeffs2 = d2,
                                          flags = flags,
                                          criteria = criteria)
            else:
                start = time.time()
                error, K1, d1, K2, d2, R, T, E, F \
                        = cv2.stereoCalibrate(objpoints,limgpoints,rimgpoints, 
                                              cameraMatrix1 = K1, distCoeffs1 = d1, 
                                              cameraMatrix2 = K2, distCoeffs2 =  d2,
                                              imageSize = frame_dims,
                                              flags = flags,
                                              criteria = criteria)
                end = time.time()
                print("Time for stereo calibration with fixed intrinsics (s): {0:f}"
                      .format(end - start)) 
        else:
            if(int(cv2.__version__ [0]) == 2):
                error, K1, d1, K2, d2, R, T, E, F \
                      = cv2.stereoCalibrate(objpoints,limgpoints,rimgpoints, 
                                            imageSize = frame_dims,
                                            flags = flags,
                                            criteria = criteria)
            else:  
                error, K1, d1, K2, d2, R, T, E, F \
                    = cv2.stereoCalibrate(objpoints,limgpoints,rimgpoints, 
                                          K1, d1, K2, d2, 
                                          imageSize = frame_dims,
                                          flags = flags,
                                          criteria = criteria)
    return error, K1, d1, K2, d2, R, T, E, F