#!/usr/bin/python3
import os
import os.path as osp
import cv2
import numpy as np
import argparse as ap
from calib import utils as cutils
from calib import io as cio


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





class CalibrateStereoVideoApplication:
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
    
            
    def run_capture(self):
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
            else:
                usable_frame_ct = self.run_capture()            
            if self.args.save_corners:
                print("Saving corners to {0:s}".format(self.full_corners_path))
                np.savez_compressed(self.full_corners_path,
                                    limgpoints=self.limgpoints,rimgpoints=self.rimgpoints, 
                                    objpoints=self.board_object_corner_set)
        print ("Total usable frames: {0:d} ({1:.3%})"
               .format(usable_frame_ct, float(usable_frame_ct)/self.total_frames))
        
    def print_output(self, error, K1, d1, K2, d2, R, T):
        print (("Final reprojection error: {0:f}\nK0:\n{1:s}\nK1:\n{2:s}\nd0:\n{3:s}\nd1:\n{4:s}\n"+
               "R:\n{5:s}\nT:\n{6:s}").format(error,str(K1),str(K2),str(d1),str(d2),str(R),str(T)))
               
    def run_calibration(self):
        print ("Calibrating for max. {0:d} iterations...".format(self.args.max_iterations))
        if(self.args.use_existing):
            path_to_calib_file = osp.join(self.args.folder, self.args.output)
        else:
            path_to_calib_file = None
        error, K1, d1, K2, d2, R, T, E, F = cutils.calibrate(self.limgpoints, self.rimgpoints, self.objpoints,  # @UnusedVariable
                                                             self.frame_dims, self.args.use_fisheye_distortion_model, 
                                                             self.args.use_8_distortion_coefficients, 
                                                             self.args.use_tangential_distortion_coefficients, 
                                                             self.args.precalibrate_solo, self.args.max_iterations, 
                                                             path_to_calib_file)
        if self.args.preview:
            l_im = cv2.imread(osp.join(self.args.folder,self.args.preview_files[0]))
            r_im = cv2.imread(osp.join(self.args.folder,self.args.preview_files[1]))
            l_im, r_im = cutils.generate_preview(K1, d1, K2, d2, R, T, l_im, r_im)
            path_l = osp.join(self.args.folder,self.args.preview_files[0][:-4] + "_rect.png")
            path_r = osp.join(self.args.folder,self.args.preview_files[1][:-4] + "_rect.png")
            cv2.imwrite(path_l, l_im)
            cv2.imwrite(path_r, r_im)
        if not self.args.skip_printing_output:
            self.print_output(error, K1, d1, K2, d2, R, T)
        if not self.args.skip_saving_output:
            cio.save_output_opencv(error, K1, d1, K2, d2, R, T, self.frame_dims, osp.join(args.folder,args.output))
        
        
        
if __name__ == "__main__":
    args = parser.parse_args()
    
    app = CalibrateStereoVideoApplication(args)
    app.gather_frame_data()
    app.run_calibration()
    
   
    
        
    
