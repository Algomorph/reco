#!/usr/bin/python
import os
import os.path as osp
import cv2
import numpy as np
import argparse as ap
import time
from calib import utils as cb


parser = ap.ArgumentParser(description='Traverse two .mp4 stereo video files and '+
                           ' calibrate the cameras based on specially selected frames within.')
parser.add_argument("-f", "--folder", help="Path to root folder to work in", 
                    required=False, default= "./")
parser.add_argument("-fn", "--frame_numbers", help="frame numbers .npz file with frame_numbers array."+
                    " If specified, program filters frame pairs based on these numbers instead of other"+
                    " criteria.",required=False, default=None)
parser.add_argument("-v", "--videos", nargs=2, help="input stereo video tuple (left, right)", 
                    required=False, default= ["left.mp4","right.mp4"])

#============== CALIBRATION PREVIEW ===============================================================#
#currently does not work due to OpenCV python bindings bug
parser.add_argument("-pf", "--preview_files", nargs=2, help="input frames to test calibration result", 
                    required=False, default= ["left.png","right.png"])
parser.add_argument("-p", "--preview", help="Test calibration result on left/right frame pair", 
                    action = "store_true", required=False)

#============== BOARD DIMENSIONS ==================================================================#
parser.add_argument("-bw", "--board_width", help="checkerboard inner corner width",required = False, 
                    default= 15, type=int)
parser.add_argument("-bh", "--board_height", help="checkerboard inner corner height",required = False,
                     default= 8, type=int)
parser.add_argument("-bs", "--board_square_size", help="checkerboard square size, in meters", 
                    required = False, type=float, default=0.0508)

#============== FRAME FILTERING CONTROLS ==========================================================#
parser.add_argument("-ft", "--sharpness_threshold", help="sharpness threshold based on variance of "+
                    "Laplacian; used to filter out frames that are too blurry (default 55.0).", 
                    type=float, required = False, default=55.0)
parser.add_argument("-fd", "--difference_threshold", help="difference threshold: minimum average "
                    +" per-pixel difference (in range [0,1.0]) between current and previous frames to "
                    +"filter out frames that are too much alike (default: 0.4)", type=float, 
                    required = False, default=0.4)
parser.add_argument("-m", "--manual_filter", help="pick which (pre-filtered)frames to use manually"+
                    " one-by-one (use 'a' key to approve)", required = False, action='store_true', 
                    default=False)
parser.add_argument("-fi", "--frame_interval", required=False, default=1, type=int,
                    help="minimal interval (in frames) between successive frames to consider for"
                    +" calibration board extraction.")
parser.add_argument("-fa", "--advanced_filtering", required=False, action="store_true", default=False,
                    help="filter frames based on camera position and orientation rather than" +
                    " basic absolute difference between pixels. With this argument, the difference"+
                    " threshold parameter is ignored.")
parser.add_argument("-aft", "--advanced_filter_target", required=False, type=int, help="Target number of "+
                    "frames to filter out" )

#============== STORAGE OF BOARD CORNER POSITIONS =================================================#
parser.add_argument("-c", "--corners_file", required = False, default="corners.npz", 
                    help="store filtered corners")
parser.add_argument("-s", "--save_corners", action='store_true', required = False, default=False)
parser.add_argument("-l", "--load_corners", action='store_true', required = False, default=False)

#============== CALIBRATION & DISTORTION MODEL CONTROLS ===========================================#
parser.add_argument("-i", "--max_iterations", help="maximum number of iterations for the stereo"+
                    " calibration (optimization) loop", type=int, required = False, default=30)
parser.add_argument("-ds", "--precalibrate_solo", help="pre-calibrate each camera individually "+
                    "first, then perform stereo calibration",action='store_true', required = False, 
                    default=False)
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
parser.add_argument("-u", "--use_existing", 
                    help="use the existing output file to initialize calibration parameters", 
                    action="store_true", required = False, default=False)

#============== FILTERED IMAGE/FRAME BACKUP & LOADING =============================================#
parser.add_argument("-if", "--filtered_image_folder", help="filtered frames"+
                    " will be saved into this folder (relative to work folder specified by --folder)", 
                    required = False, default="frames")
parser.add_argument("-is", "--save_images", action='store_true', required = False, default= False)
parser.add_argument("-il", "--load_images", action='store_true', required = False, default= False)

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

def automatic_filter(lframe,rframe,lframe_prev,rframe_prev,sharpness_threshold, difference_threshold):
    sharpness = min(cv2.Laplacian(lframe, cv2.CV_64F).var(), cv2.Laplacian(rframe, cv2.CV_64F).var())
    #compare left frame to the previous left **filtered** one
    ldiff = np.sum(abs(lframe_prev - lframe)) / diff_div
    
    if sharpness < args.sharpness_threshold or ldiff < args.difference_threshold:
        return False, None, None
    
    lfound,lcorners = cv2.findChessboardCorners(lframe,board_dims)
    rfound,rcorners = cv2.findChessboardCorners(rframe,board_dims)
    if not (lfound and rfound):
        return False, None, None
    
    return True, lcorners, rcorners

if __name__ == "__main__":
    args = parser.parse_args()
    left_vid = osp.join(args.folder,args.videos[0])
    right_vid = osp.join(args.folder,args.videos[1])
    
    if not osp.isfile(left_vid):
        raise ValueError("No video file found at {0:s}".format(left_vid))
    if not osp.isfile(right_vid):
        raise ValueError("No video file found at {0:s}".format(right_vid))
    
    l_vid_name = args.videos[0][:-4] 
    r_vid_name = args.videos[1][:-4]
    
    print "Calibrating based on files: "
    print(left_vid)
    print(right_vid)
    
    if args.save_images or args.load_images:
        full_frame_folder_path = osp.join(args.folder,args.filtered_image_folder)
        #if filtered frame folder is specified but doesn't exist, create it
        if not os.path.exists(full_frame_folder_path):
            os.makedirs(full_frame_folder_path)
            
    if args.frame_numbers:
        path = osp.join(args.folder, args.frame_numbers)
        print "Loading frame numbers from \"" + path + "\"."
        npzfile = np.load(path)
        frame_numbers = set(npzfile["frame_numbers"])

    left_cap = cv2.VideoCapture(left_vid)
    right_cap = cv2.VideoCapture(right_vid)
    
    limgpoints = []
    rimgpoints = []
    objpoints = []

    board_dims = (args.board_width,args.board_height)
    objp = np.zeros((args.board_height*args.board_width,1,3), np.float32)
    objp[:,:,:2] = np.indices(board_dims).T.reshape(-1, 1, 2)
    #convert square sizes to meters
    objp *= args.board_square_size
    
    lret, lframe = left_cap.read()
    rret, rframe = right_cap.read()
    frame_dims = lframe.shape[:-1]
    
    usable_frame_ct = 0
    i_frame = 0
    criteria_subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    if(args.load_corners):    
        limgpoints,rimgpoints,objpoints, usable_frame_ct = cb.load_corners(path)
        i_frame = usable_frame_ct
    elif(args.load_images):
        print "Loading frames from '{0:s}'".format(full_frame_folder_path)
        files = [f for f in os.listdir(full_frame_folder_path) if osp.isfile(osp.join(full_frame_folder_path,f)) and f.endswith(".png")]
        files.sort()
        #assume matching numbers in corresponding left & right files
        lfiles = [f for f in files if f.startswith(l_vid_name)]
        rfiles = [f for f in files if f.startswith(r_vid_name)]
        #assume all are usable
        for ix_pair in xrange(min(len(lfiles),len(rfiles))):
            lframe = cv2.imread(osp.join(full_frame_folder_path,lfiles[ix_pair]))
            rframe = cv2.imread(osp.join(full_frame_folder_path,rfiles[ix_pair]))
            lfound,lcorners = cv2.findChessboardCorners(lframe,board_dims)
            rfound,rcorners = cv2.findChessboardCorners(rframe,board_dims)
            lgrey = cv2.cvtColor(lframe,cv2.COLOR_BGR2GRAY)
            rgrey = cv2.cvtColor(rframe,cv2.COLOR_BGR2GRAY)
            cv2.cornerSubPix(lgrey, lcorners, (11,11),(-1,-1),criteria_subpix)
            cv2.cornerSubPix(rgrey, rcorners, (11,11),(-1,-1),criteria_subpix)
            limgpoints.append(lcorners)
            rimgpoints.append(rcorners)
            objpoints.append(objp)
            usable_frame_ct+=1
            i_frame+=1
        if args.save_corners:
            path = args.folder + os.path.sep + args.corners_file
            print "Saving corners..."
            np.savez_compressed(path,limgpoints=limgpoints,rimgpoints=rimgpoints)
    else:
        #init capture
        cont_cap = lret and rret
        lframe_prev = np.zeros(lframe.shape, lframe.dtype)
        rframe_prev = np.zeros(rframe.shape, rframe.dtype)
        diff_div = lframe.shape[0] * lframe.shape[1] * 3 * 256.0
        report_interval = 10
        #for keeping track of the difference between frames
        diff = 0 
        
        
        while(cont_cap):
            sharpness = min(cv2.Laplacian(lframe, cv2.CV_64F).var(), cv2.Laplacian(rframe, cv2.CV_64F).var())
            #compare left frame to the previous left **filtered** one
            ldiff = np.sum(abs(lframe_prev - lframe)) / diff_div
            
            if not args.frame_numbers or i_frame in frame_numbers:
                add_corners, lcorners, rcorners = automatic_filter(lframe, rframe, lframe_prev, rframe_prev, 
                                                                   args.sharpness_threshold, args.difference_threshold)
                
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
                    lgrey = cv2.cvtColor(lframe,cv2.COLOR_BGR2GRAY)
                    rgrey = cv2.cvtColor(rframe,cv2.COLOR_BGR2GRAY)
                    cv2.cornerSubPix(lgrey, lcorners, (11,11),(-1,-1),criteria_subpix)
                    cv2.cornerSubPix(rgrey, rcorners, (11,11),(-1,-1),criteria_subpix)
                    
                    #save frames if filtered frame folder was specified
                    if(args.save_images):
                        lfname = (full_frame_folder_path + os.path.sep + l_vid_name
                            + "{0:04d}".format(i_frame) + ".png")
                        rfname = (full_frame_folder_path + os.path.sep + r_vid_name
                            + "{0:04d}".format(i_frame) + ".png")
                        cv2.imwrite(lfname, lframe)
                        cv2.imwrite(rfname, rframe)
                        
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
            np.savez_compressed(path,limgpoints=limgpoints,rimgpoints=rimgpoints, objpoints=objp)
        
    left_cap.release()
    right_cap.release()
    
    print "Total usable frames: " + str(usable_frame_ct) + " (" + str(usable_frame_ct * 100.0/(i_frame)) + " %)"
    print "Calibrating for max. " +str(args.max_iterations) + " iterations...."
   
    if(args.use_existing):
        path_to_calib_file = osp.join(args.folder, args.output)
    else:
        path_to_calib_file = None
    error, K1, d1, K2, d2, R, T, E, F = cb.calibrate(limgpoints, rimgpoints, objpoints, 
                                                     frame_dims, args.use_fisheye_distortion_model, 
                                                     args.use_8_distortion_coefficients, 
                                                     args.use_tangential_distortion_coefficients, 
                                                     args.precalibrate_solo, args.max_iterations, 
                                                     path_to_calib_file)
    if not args.skip_printing_output:
        print_output(error, K1, d1, K2, d2, R, T)
    if not args.skip_saving_output:
        cb.save_output_opencv(error, K1, d1, K2, d2, R, T, frame_dims, osp.join(args.folder,args.output))
        
    if args.preview:
        l_im = cv2.imread(osp.join(args.folder,args.preview_files[0]))
        r_im = cv2.imread(osp.join(args.folder,args.preview_files[1]))
        l_im, r_im = cb.generate_preview(K1, d1, K2, d2, R, T, l_im, r_im)
        path_l = osp.join(args.folder,args.preview_files[0][:-4] + "_rect.png")
        path_r = osp.join(args.folder,args.preview_files[1][:-4] + "_rect.png")
        cv2.imwrite(path_l, l_im)
        cv2.imwrite(path_r, r_im)
