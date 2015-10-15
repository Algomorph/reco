#!/usr/bin/python
import os
import sys
import os.path as osp
import cv2
import numpy as np
import argparse as ap
from lxml import etree
import re

parser = ap.ArgumentParser(description='Traverse all .mp4 video files in the specified folder and'+
                           ' pick out frames to export as images.')
parser.add_argument("-f", "--folder", help="Folder to work in", 
                    required=False, default= ["./"])
parser.add_argument("-v", "--videos", nargs=2, help="Input stereo video tuple (left, right)", 
                    required=False, default= ["left.mp4","right.mp4"])
parser.add_argument("-bw", "--board_width", help="checkerboard inner corner width",required = False, 
                    default= 15)
parser.add_argument("-bh", "--board_height", help="checkerboard inner corner height",required = False,
                     default= 8)
parser.add_argument("-t", "--sharpness_threshold", help="sharpness threshold based on variance of "+
                    "Laplacian; used to filter out frames that are too blurry.", 
                    type=float, required = False, default=55.0)
parser.add_argument("-d", "--difference_threshold", help="difference threshold: maximum average "
                    +" per-pixel difference (in range [0,1.0]) between current and previous frames to "
                    +"filter out frames that are too much alike", type=float, required = False, default=0.4)
parser.add_argument("-c", "--corners_file", required = False, default="corners.npz")
parser.add_argument("-s", "--save_corners", action='store_true', required = False, default=False)
parser.add_argument("-l", "--load_corners", action='store_true', required = False, default=False)
parser.add_argument("-ds", "--precalibrate_solo", action='store_true', required = False, default=False)
parser.add_argument("-i", "--max_iterations", help="maximum number of iterations for the stereo"+
                    " calibration (optimization) loop", type=int, required = False, default=30)
parser.add_argument("-d8", "--use_8_distortion_coefficients", action='store_true', required = False, 
                    default=False)
parser.add_argument("-dt", "--use_tangential_distortion_coefficients", action='store_true', 
                    required = False, default=False)
parser.add_argument("-df", "--use_fisheye_distortion_model", action='store_true', 
                    required = False, default=False)
parser.add_argument("-sp", "--skip_printing_output", action='store_true', required = False, default= False)
parser.add_argument("-ss", "--skip_saving_output", action='store_true', required = False, default= False)
parser.add_argument("-o", "--output", help="output file to store calibration results", 
                    required = False, default="cvcalib.xml")
parser.add_argument("-of", "--filtered_frame_folder", help="If specified, filtered frames"+
                    " will be saved into this folder (relative to work folder specified by --folder)", 
                    required = False, default=None)
parser.add_argument("-m", "--manual_filter", help="pick which (pre-filtered)frames to use manually"+
                    " one-by-one (use 'a' key to approve)", required = False, action='store_true', 
                    default=False)


def print_output(error, K1, d1, K2, d2, R, T):
    print "Final reprojection error: " + str(error)
    print "K0:"
    print K1
    print "K1:"
    print K2
    print "d0:"
    print d1
    print "d1:"
    print d2
    print "R:"
    print R
    print "T:"
    print T
    
def opencv_matrix_xml_element(root, mat,name):
    mat_element = etree.SubElement(root, name, attrib={"type_id":"opencv-matrix"})
    rows_elem = etree.SubElement(mat_element, "rows")
    rows_elem.text = str(mat.shape[0])
    cols_elem = etree.SubElement(mat_element, "cols")
    cols_elem.text = str(mat.shape[1])
    dt_elem = etree.SubElement(mat_element, "dt")
    if(mat.dtype == np.dtype('float64')):
        dt_elem.text = "d"
    elif(mat.dtype == np.dtype("float32")):
        dt_elem.text = "f"
    else:
        raise ValueError("dtype " + str(mat.dtype) + "not supported")
    
    data_elem = etree.SubElement(mat_element, "data")
    data_string = str(mat.flatten()).replace("\n","").replace("[","").replace("]","")
    data_string = re.sub("\s+"," ",data_string)
    data_elem.text = data_string
    return mat_element
    
def save_output_opencv(error, K1, d1, K2, d2, R, T, dims, file_path):
    root = etree.Element("opencv_storage")
    width_element = etree.SubElement(root, "width")
    width_element.text = str(dims[1])
    width_element = etree.SubElement(root, "height")
    width_element.text = str(dims[0])
    K1_element = opencv_matrix_xml_element(root, K1, "K1")
    d1_element = opencv_matrix_xml_element(root, d1, "d1")
    K2_element = opencv_matrix_xml_element(root, K2, "K2")
    d2_element = opencv_matrix_xml_element(root, d2, "d2")
    R_element = opencv_matrix_xml_element(root, R, "R")
    T_element = opencv_matrix_xml_element(root, T, "T")
    error_element = etree.SubElement(root, "reprojection_error")
    error_element.text = str(error)
    et = etree.ElementTree(root)
    with open(file_path,'w') as f:
        et.write(f,encoding='utf-8',xml_declaration=True, pretty_print=True)
    
    
if __name__ == "__main__":
    args = parser.parse_args()
    left_vid = args.folder + os.path.sep + args.videos[0]
    right_vid = args.folder + os.path.sep +  args.videos[1]
    
    print "Capturing frames from files: "
    print(left_vid)
    print(right_vid)
    
    if args.filtered_frame_folder:
        full_frame_folder_path = args.folder + os.path.sep + args.filtered_frame_folder
        #if filtered frame folder is specified but doesn't exist, create it
        if not os.path.exists(full_frame_folder_path):
            os.makedirs(full_frame_folder_path)
    
    left_cap = cv2.VideoCapture(left_vid)
    right_cap = cv2.VideoCapture(right_vid)
    
    board_dims = (args.board_width,args.board_height)
    
    limgpoints = []
    rimgpoints = []
    objpoints = []

    objp = np.zeros((args.board_height*args.board_width,1,3), np.float32)
    objp[:,:,:2] = np.indices(board_dims).T.reshape(-1, 1, 2)
    #convert inches to meters
    objp *= 0.0508
    
    lret, lframe = left_cap.read()
    rret, rframe = right_cap.read()
    frame_dims = lframe.shape[:-1]
    
    
    if(args.load_corners):
        path = args.folder + os.path.sep + args.corners_file
        npzfile = np.load(path)
        limgpoints = npzfile['limgpoints']
        rimgpoints = npzfile['rimgpoints']
        usable_frame_ct = len(limgpoints)
        for i_frame in xrange(usable_frame_ct):
            objpoints.append(objp)
    else:
        #init capture
        cont_cap = lret and rret
        lframe_prev = np.zeros(lframe.shape, lframe.dtype)
        rframe_prev = np.zeros(rframe.shape, rframe.dtype)
        diff_div = lframe.shape[0] * lframe.shape[1] * 3 * 256.0
        usable_frame_ct = 0
        report_interval = 10
        i_frame = 0
        #for keeping track of the difference between frames
        diff = 0 
        
        criteria_subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        while(cont_cap):
            sharpness = min(cv2.Laplacian(lframe, cv2.CV_64F).var(), cv2.Laplacian(rframe, cv2.CV_64F).var())
            #compare left frame to the previous left **filtered** one
            ldiff = np.sum(abs(lframe_prev - lframe)) / diff_div
           
            if sharpness >= args.sharpness_threshold and ldiff > args.difference_threshold:
                
                lfound,lcorners = cv2.findChessboardCorners(lframe,board_dims)
                rfound,rcorners = cv2.findChessboardCorners(rframe,board_dims)
                lgrey = cv2.cvtColor(lframe,cv2.COLOR_BGR2GRAY)
                rgrey = cv2.cvtColor(rframe,cv2.COLOR_BGR2GRAY)
                            
                if lfound and rfound:
                    add_corners = True
                    if args.manual_filter:
                        combo = np.hstack((lframe, rframe))
                        cv2.imshow("frame", combo)
                        key = cv2.waitKey(0) & 0xFF
                        add_corners = (key == ord('a'))
                        cv2.destroyWindow("frame")
                        
                    if add_corners:
                        usable_frame_ct += 1
                        if(usable_frame_ct % report_interval == 0):
                            print "Usable frames: " + str(usable_frame_ct) + " (" + str(usable_frame_ct * 100.0/(i_frame+1.0)) + " %)"
                        cv2.cornerSubPix(lgrey, lcorners, (11,11),(-1,-1),criteria_subpix)
                        cv2.cornerSubPix(rgrey, rcorners, (11,11),(-1,-1),criteria_subpix)
                        
                        #save frames if filtered frame folder was specified
                        if(args.filtered_frame_folder):
                            cv2.imwrite(full_frame_folder_path + os.path.sep + left_vid[:-4] 
                                        + "{0:04d}".format(i_frame) + ".png", lframe)
                            cv2.imwrite(full_frame_folder_path + os.path.sep + right_vid[:-4] 
                                        + "{0:04d}".format(i_frame) + ".png", rframe)
                            
                        limgpoints.append(lcorners)
                        rimgpoints.append(rcorners)
                        objpoints.append(objp)
                    
                    #log last usable **filtered** frame
                    lframe_prev = lframe
                    rframe_prev = rframe
            
            i_frame += 1
            lret, lframe = left_cap.read()
            rret, rframe = right_cap.read()
            cont_cap = lret and rret
            if args.manual_filter and key == 27:
                cont_cap = False
                
        if(args.manual_filter):
            cv2.destroyAllWindows()
            
        if args.save_corners:
            path = args.folder + os.path.sep + args.corners_file
            print "Saving corners..."
            np.savez_compressed(path,limgpoints=limgpoints,rimgpoints=rimgpoints)
        
    left_cap.release()
    right_cap.release()
    
    print "Total usable frames: " + str(usable_frame_ct) + " (" + str(usable_frame_ct * 100.0/(i_frame+1.0)) + " %)"
    print "Calibrating for max. " +str(args.max_iterations) + " iterations...."
    identity = np.eye(3,3,np.float32)
    
    criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, args.max_iterations, 2.2204460492503131e-16)
    flags = 0
    
    if args.use_fisheye_distortion_model:
        #objpoints_np = np.array(objpoints)
        #print objpoints_np.shape
        #print np.transpose(objpoints_np, (0,2,1)).shape
        blank = np.zeros(4,np.float32)
        error, K1, d1, K2, d2, R, T \
            = cv2.fisheye.stereoCalibrate(objpoints, limgpoints, rimgpoints, 
                                          identity, blank, identity, blank, 
                                          imageSize=frame_dims,
                                          flags=flags,
                                          criteria=criteria)
    else:
        blank = np.zeros(8,np.float32)
        if not args.use_tangential_distortion_coefficients:
            flags += cv2.CALIB_ZERO_TANGENT_DIST
        if args.use_8_distortion_coefficients:
            flags += cv2.CALIB_RATIONAL_MODEL
        
        if(args.precalibrate_solo):
            err1, K1, d1, rvecs, tvecs = cv2.calibrateCamera(objpoints, limgpoints, 
                                                       frame_dims, identity, blank, flags=flags,criteria = criteria)
            print "K1 solo:"
            print K1
            print "d1 solo:"
            print d1
            print "solo 1 err:"
            print err1
            err2, K2, d2, rvecs, tvecs = cv2.calibrateCamera(objpoints, rimgpoints, 
                                                       frame_dims, identity, blank, flags=flags,criteria = criteria)
            print "K2 solo:"
            print K2
            print "d2 solo:"
            print d2
            print "solo 2 err:"
            print err2
            flags += cv2.CALIB_FIX_INTRINSIC
            #flags += cv2.CALIB_USE_INTRINSIC_GUESS
            print "Flags:"
            print flags
            error, K1_2, d1, K2_2, d2, R, T, E, F \
                    = cv2.stereoCalibrate(objpoints,limgpoints,rimgpoints, 
                                          K1, d1, K2, d2, 
                                          imageSize = frame_dims,
                                          flags = flags,
                                          criteria = criteria)
        else:
            error, K1, d1, K2, d2, R, T, E, F \
                = cv2.stereoCalibrate(objpoints,limgpoints,rimgpoints, 
                                      identity, blank, identity2, blank, 
                                      imageSize = frame_dims,
                                      flags = flags,
                                      criteria = criteria)
            
    if not args.skip_printing_output:
        print_output(error, K1, d1, K2, d2, R, T)
    if not args.skip_saving_output:
        save_output_opencv(error, K1, d1, K2, d2, R, T, frame_dims, args.folder + os.path.sep + args.output)
    
