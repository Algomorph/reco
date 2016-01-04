#!/usr/bin/python3
import os
import os.path as osp
import cv2
import numpy as np
import argparse as ap
from calib import utils as cutils
from calib import io as cio


parser = ap.ArgumentParser(description='Traverse two .mp4 stereo video files and '+
                           ' stereo_calibrate the cameras based on specially selected frames within.')
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
parser.add_argument("-ds", "--precalibrate_solo", help="pre-stereo_calibrate each camera individually "+
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





class CalibrateStereoVideoApplication:
    min_frames_to_calibrate = 4
    def __init__(self,args):
        self.left_vid = osp.join(args.folder,args.videos[0])
        self.right_vid = osp.join(args.folder,args.videos[1])
        if not osp.isfile(self.left_vid):
            raise ValueError("No video file found at {0:s}".format(self.left_vid))
        if not osp.isfile(self.right_vid):
            raise ValueError("No video file found at {0:s}".format(self.right_vid))
        self.l_vid_name = args.videos[0][:-4] 
        self.r_vid_name = args.videos[1][:-4]
        self.left_cap = cv2.VideoCapture(self.left_vid)
        self.right_cap = cv2.VideoCapture(self.right_vid)
        
        self.full_frame_folder_path = osp.join(args.folder,args.filtered_image_folder)
        #if image folder doesn't yet exist, create it
        if args.save_images and not os.path.exists(self.full_frame_folder_path):
            os.makedirs(self.full_frame_folder_path)
        self.full_corners_path = osp.join(args.folder,args.corners_file)
        self.limgpoints = []
        self.rimgpoints = []
        self.objpoints = []
        self.board_dims = (args.board_width,args.board_height)
        self.board_object_corner_set = np.zeros((args.board_height*args.board_width,1,3), np.float32)
        self.board_object_corner_set[:,:,:2] = np.indices(self.board_dims).T.reshape(-1, 1, 2)
        self.board_object_corner_set *= args.board_square_size
        if args.frame_numbers:
            path = osp.join(args.folder, args.frame_numbers)
            print("Loading frame numbers from \"{0:s}\"".format(path))
            npzfile = np.load(path)
            self.frame_numbers = set(npzfile["frame_numbers"])
        self.criteria_subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.frame_dims = (int(self.left_cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                           int(self.left_cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
        self.total_frames = self.left_cap.get(cv2.CAP_PROP_FRAME_COUNT)
        self.pixel_difference_factor = 1.0 / (self.board_dims[0] * self.board_dims[1] * 3 * 256.0)
        self.args = args
        
    def __del__(self):
        self.left_cap.release()
        self.right_cap.release()
    
    def __automatic_filter(self, l_frame,r_frame,
                     lframe_prev,rframe_prev,
                     difference_factor):
        sharpness = min(cv2.Laplacian(l_frame, cv2.CV_64F).var(), 
                        cv2.Laplacian(r_frame, cv2.CV_64F).var())
        #compare left frame to the previous left **filtered** one
        ldiff = np.sum(abs(lframe_prev - l_frame)) * self.pixel_difference_factor
        
        if sharpness < self.args.sharpness_threshold or ldiff < self.args.difference_threshold:
            return False, None, None
        
        lfound,lcorners = cv2.findChessboardCorners(l_frame,self.board_dims)
        rfound,rcorners = cv2.findChessboardCorners(r_frame,self.board_dims)
        if not (lfound and rfound):
            return False, None, None
        
        return True, lcorners, rcorners

    def load_frame_images(self):
        print("Loading frames from '{0:s}'".format(self.full_frame_folder_path))
        files = [f for f in os.listdir(self.full_frame_folder_path) 
                 if osp.isfile(osp.join(self.full_frame_folder_path,f)) and f.endswith(".png")]
        files.sort()
        #assume matching numbers in corresponding left & right files
        lfiles = [f for f in files if f.startswith(self.l_vid_name)]
        rfiles = [f for f in files if f.startswith(self.r_vid_name)]
        #assume all are usable
        usable_frame_ct = 0
        for ix_pair in range(min(len(lfiles),len(rfiles))):
            l_frame = cv2.imread(osp.join(self.full_frame_folder_path,lfiles[ix_pair]))
            r_frame = cv2.imread(osp.join(self.full_frame_folder_path,rfiles[ix_pair]))
            lfound,lcorners = cv2.findChessboardCorners(l_frame,self.board_dims)
            rfound,rcorners = cv2.findChessboardCorners(r_frame,self.board_dims)
            if not (lfound):
                raise ValueError("Could not find corners in image '{0:s}'".format(lfiles[ix_pair]))
            if not (rfound):
                raise ValueError("Could not find corners in image '{0:s}'".format(rfiles[ix_pair]))
            lgrey = cv2.cvtColor(l_frame,cv2.COLOR_BGR2GRAY)
            rgrey = cv2.cvtColor(r_frame,cv2.COLOR_BGR2GRAY)
            cv2.cornerSubPix(lgrey, lcorners, (11,11),(-1,-1),self.criteria_subpix)
            cv2.cornerSubPix(rgrey, rcorners, (11,11),(-1,-1),self.criteria_subpix)
            self.limgpoints.append(lcorners)
            self.rimgpoints.append(rcorners)
            self.objpoints.append(self.board_object_corner_set)
            usable_frame_ct+=1
        return usable_frame_ct
    
    
    def bootstrap(self, num_frames_for_bootstrap = 20):
        '''
        pick out a few frames and stereo_calibrate based on those
        '''
        
        
        #assume a relatively uniform distribution of camera angles and positions
        bootstrap_frame_interval = self.total_frames / num_frames_for_bootstrap
        
        start_seek_frame = 0
        
        bootstrap_limgpts = []
        bootstrap_rimgpts = []
        bootstrap_objectpts = []
        bootstrap_frame_ct = 0
        
        preview = True
        
        if(preview):
            screen_width = 1920
            screen_height = 1080
            top_offset = 30
            launcher_offset = 100
            cv2.namedWindow("left", flags = cv2.WINDOW_KEEPRATIO)
            cv2.namedWindow("right", flags = cv2.WINDOW_KEEPRATIO)
            cv2.moveWindow("left", launcher_offset, top_offset)
            cv2.moveWindow("right", screen_width, top_offset)
            cv2.resizeWindow("left", screen_width-launcher_offset, screen_height-top_offset)
            cv2.resizeWindow("right", screen_width, screen_height-top_offset)
        
        while start_seek_frame < self.total_frames:
            got_bootstrap_frame = False
            i_frame = start_seek_frame
            self.left_cap.set(cv2.CAP_PROP_POS_FRAMES,i_frame)
            self.right_cap.set(cv2.CAP_PROP_POS_FRAMES,i_frame)
            lret, l_frame = self.left_cap.read()
            rret, r_frame = self.right_cap.read()
            continue_capture = lret and rret
            
            while not got_bootstrap_frame and continue_capture:
                #try sampling a frame
                sharpness_left = cv2.Laplacian(l_frame, cv2.CV_64F).var()
                sharpness_right = cv2.Laplacian(r_frame, cv2.CV_64F).var()  
                sharpness = min(sharpness_left,sharpness_right) 
                
                if(sharpness > self.args.sharpness_threshold or preview):
                    lfound,lcorners = cv2.findChessboardCorners(l_frame,self.board_dims)
                    rfound,rcorners = cv2.findChessboardCorners(r_frame,self.board_dims)
                    if (lfound and rfound):
                        lgrey = cv2.cvtColor(l_frame,cv2.COLOR_BGR2GRAY)
                        rgrey = cv2.cvtColor(r_frame,cv2.COLOR_BGR2GRAY)
                        cv2.cornerSubPix(lgrey, lcorners, (11,11),(-1,-1),self.criteria_subpix)
                        cv2.cornerSubPix(rgrey, rcorners, (11,11),(-1,-1),self.criteria_subpix)
                        if(preview):
                            cv2.drawChessboardCorners(l_frame, self.board_dims, lcorners, lfound)
                            cv2.drawChessboardCorners(r_frame, self.board_dims, rcorners, rfound)
                            print("Sharpness left: {0:f}".format(sharpness_left))
                            print("Sharpness right: {0:f}".format(sharpness_right))
                            cv2.imshow("left", l_frame)
                            cv2.imshow("right", r_frame)
                            key = cv2.waitKey(0)
                            if(key == 27):
                                preview = False
                                cv2.destroyAllWindows()
                        bootstrap_limgpts.append(lcorners)
                        bootstrap_rimgpts.append(rcorners)
                        bootstrap_objectpts.append(self.board_object_corner_set)
                        bootstrap_frame_ct += 1
                        got_bootstrap_frame = True
                i_frame +=1
                lret, l_frame = self.left_cap.read()
                rret, r_frame = self.right_cap.read()
                continue_capture = lret and rret
            #advance to next sampling interval
            start_seek_frame += bootstrap_frame_interval
        
        print("Got total bootstrap frames: {0:d}".format(bootstrap_frame_ct))
        
    
    def run_capture_advanced(self):
        self.bootstrap()
        return 0
        
        
            
            
    
            
    def run_capture_basic(self):
        #just in case we're running capture again
        self.left_cap.set(cv2.CAP_PROP_POS_FRAMES,0.0)
        self.right_cap.set(cv2.CAP_PROP_POS_FRAMES,0.0)
        
        #init capture
        lret, l_frame = self.left_cap.read()
        rret, r_frame = self.right_cap.read()
        continue_capture = lret and rret
        lframe_prev = np.zeros(l_frame.shape, l_frame.dtype)
        rframe_prev = np.zeros(r_frame.shape, r_frame.dtype)
        usable_frame_ct = 0
        i_frame = 0
         
        report_interval = 10

        while(continue_capture):
            if not self.args.frame_numbers or i_frame in self.frame_numbers:
                add_corners, lcorners, rcorners = self.__automatic_filter(l_frame, r_frame, 
                                                                          lframe_prev, rframe_prev)
                if self.args.manual_filter:
                    combo = np.hstack((l_frame, r_frame))
                    cv2.imshow("frame", combo)
                    key = cv2.waitKey(0) & 0xFF
                    add_corners = (key == ord('a'))
                    cv2.destroyWindow("frame")
                      
                if add_corners:
                    usable_frame_ct += 1
                    if(usable_frame_ct % report_interval == 0):
                        print ("Usable frames: {0:i} ({1:.3%})"
                               .format(usable_frame_ct, float(usable_frame_ct)/(i_frame+1)))
                    lgrey = cv2.cvtColor(l_frame,cv2.COLOR_BGR2GRAY)
                    rgrey = cv2.cvtColor(r_frame,cv2.COLOR_BGR2GRAY)
                    cv2.cornerSubPix(lgrey, lcorners, (11,11),(-1,-1),self.criteria_subpix)
                    cv2.cornerSubPix(rgrey, rcorners, (11,11),(-1,-1),self.criteria_subpix)
                      
                    #save frames if filtered frame folder was specified
                    if(self.args.save_images):
                        lfname = (osp.join(self.full_frame_folder_path,
                                           "{0:s}{1:04d}{2:s}".format(self.l_vid_name,
                                                                      i_frame,".png")))
                        rfname = (osp.join(self.full_frame_folder_path,
                                           "{0:s}{1:04d}{2:s}".format(self.r_vid_name,
                                                                      i_frame,".png")))
                        cv2.imwrite(lfname, l_frame)
                        cv2.imwrite(rfname, r_frame)
                          
                    self.limgpoints.append(lcorners)
                    self.rimgpoints.append(rcorners) 
                    self.objpoints.append(self.board_object_corner_set)
                  
                    #log last usable **filtered** frame
                    lframe_prev = l_frame
                    rframe_prev = r_frame
      
            i_frame += 1
            lret, l_frame = self.left_cap.read()
            rret, r_frame = self.right_cap.read()
            continue_capture = lret and rret
            if self.args.manual_filter and key == 27:
                continue_capture = False
        if self.args.manual_filter:
            cv2.destroyAllWindows()
        return usable_frame_ct
        
    def gather_frame_data(self):
        self.limgpoints = []
        self.rimgpoints = []
        self.objpoints = []
        print("Gathering frame data...")
        usable_frame_ct = 0
        if(self.args.load_corners):
            print("Loading corners from {0:s}".format(self.full_corners_path))    
            self.limgpoints,self.rimgpoints,self.objpoints, usable_frame_ct =\
            cio.load_corners(self.full_corners_path)
            usable_frame_ct = len(self.objpoints)
        else:
            if(self.args.load_images):
                usable_frame_ct = self.load_frame_images()
            elif(self.args.advanced_filtering):
                usable_frame_ct = self.run_capture_advanced()
            else:
                usable_frame_ct = self.run_capture_basic()            
            if self.args.save_corners:
                print("Saving corners to {0:s}".format(self.full_corners_path))
                np.savez_compressed(self.full_corners_path,
                                    limgpoints=self.limgpoints,rimgpoints=self.rimgpoints, 
                                    object_point_set=self.board_object_corner_set)
        print ("Total usable frames: {0:d} ({1:.3%})"
               .format(usable_frame_ct, float(usable_frame_ct)/self.total_frames))
        self.usable_frame_count = usable_frame_ct
               
    def run_calibration(self):
        min_frames = CalibrateStereoVideoApplication.min_frames_to_calibrate
        if self.usable_frame_count < min_frames:
            print("Not enough usable frames to calibrate."+
                  " Need at least {0:d}, got {1:d}".format(min_frames,self.usable_frame_count))
            return
        print ("Calibrating for max. {0:d} iterations...".format(self.args.max_iterations))
        if(self.args.use_existing):
            path_to_calib_file = osp.join(self.args.folder, self.args.output)
        else:
            path_to_calib_file = None
        calibration_result = cutils.stereo_calibrate(self.limgpoints, self.rimgpoints, self.objpoints,
                                                     self.frame_dims, self.args.use_fisheye_distortion_model, 
                                                     self.args.use_8_distortion_coefficients, 
                                                     self.args.use_tangential_distortion_coefficients, 
                                                     self.args.precalibrate_solo, self.args.max_iterations, 
                                                     path_to_calib_file)
        if self.args.preview:
            l_im = cv2.imread(osp.join(self.args.folder,self.args.preview_files[0]))
            r_im = cv2.imread(osp.join(self.args.folder,self.args.preview_files[1]))
            l_im, r_im = cutils.generate_preview(calibration_result, l_im, r_im)
            path_l = osp.join(self.args.folder,self.args.preview_files[0][:-4] + "_rect.png")
            path_r = osp.join(self.args.folder,self.args.preview_files[1][:-4] + "_rect.png")
            cv2.imwrite(path_l, l_im)
            cv2.imwrite(path_r, r_im)
        if not self.args.skip_printing_output:
            print(calibration_result)
        if not self.args.skip_saving_output:
            cio.save_opencv_stereo_calibration(osp.join(args.folder,args.output), calibration_result)
        
        
        
if __name__ == "__main__":
    args = parser.parse_args()
    
    app = CalibrateStereoVideoApplication(args)
    app.gather_frame_data()
    app.run_calibration()
    
   
    
        
    
