#!/usr/bin/python3
import os
import os.path as osp
import cv2
import numpy as np
import argparse as ap
from calib import utils as cutils
from calib import io as cio
from calib.data import CameraInfo
import datetime
import sys
import re
from enum import Enum
import ConfigParser





class CalibrateVideoApplication:
    min_frames_to_calibrate = 4
    def __init__(self,args):
        self.camera = CameraInfo(args.folder,args.videos[0], 0)
        
        if(len(args.videos) > 1):
            self.cameras = [self.camera, CameraInfo(args.folder, args.videos[1], 1)]
            self.__automatic_filter_basic = self.__automatic_filter_basic_stereo
            self.__automatic_filter = self.__automatic_filter_stereo
            if(len(args.preview_files) != len(args.videos)):
                raise ValueError("There must be two preview file arguments passed in for stereo calibration.")
            self.total_frames = min(self.cameras[0].frame_count,self.cameras[1].frame_count)
            if(self.cameras[0].frame_dims != self.cameras[1].frame_dims):
                raise ValueError("The videos must have the same resolution.")
        else:
            self.cameras = [self.camera]
            self.__automatic_filter_basic = self.__automatic_filter_basic_mono
            self.__automatic_filter = self.__automatic_filter_mono
            self.total_frames = self.camera.frame_count
        
        self.full_frame_folder_path = osp.join(args.folder,args.filtered_image_folder)
        #if image folder (for captured frames) doesn't yet exist, create it
        if args.save_images and not os.path.exists(self.full_frame_folder_path):
            os.makedirs(self.full_frame_folder_path)
        self.full_corners_path = osp.join(args.folder,args.corners_file)
        
        #set up board (3D object points of checkerboard used for calibration)
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
            
        self.criteria_subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
        self.frame_dims = self.camera.frame_dims
        
        
        self.pixel_difference_factor = 1.0 / (self.board_dims[0] * self.board_dims[1] * 3 * 256.0)
        if(args.use_existing):
            self.path_to_calib_file = osp.join(self.args.folder, self.args.output)
        else:
            self.path_to_calib_file = None
        if(args.output is None):
            args.output = "calib{0:s}.xml".format(re.sub(r"-|:","",
                                                         str(datetime.datetime.now())[:-7])
                                                  .replace(" ","-"))
        self.args = args
    
    def __automatic_filter_stereo(self):
        l_frame = self.cameras[0].frame
        lframe_prev = self.cameras[0].previous_frame
        r_frame = self.cameras[1].frame

        sharpness = min(cv2.Laplacian(l_frame, cv2.CV_64F).var(), 
                        cv2.Laplacian(r_frame, cv2.CV_64F).var())
        
        #compare left frame to the previous left **filtered** one
        ldiff = np.sum(abs(lframe_prev - l_frame)) * self.pixel_difference_factor
        
        if sharpness < self.args.sharpness_threshold or ldiff < self.args.difference_threshold:
            return False
        
        lfound,lcorners = cv2.findChessboardCorners(l_frame,self.board_dims)
        rfound,rcorners = cv2.findChessboardCorners(r_frame,self.board_dims)
        if not (lfound and rfound):
            return False
        
        self.cameras[0].current_corners = lcorners
        self.cameras[1].current_corners = rcorners
        
        return True
    
    def __automatic_filter_mono(self):
        frame = self.camera.frame
        frame_prev = self.camera.previous_frame
        sharpness = cv2.Laplacian(frame,cv2.CV_64F).var()
        if sharpness < self.args.sharpness_threshold:
            return False
        #compare left frame to the previous left **filtered** one
        ldiff = np.sum(abs(frame_prev - frame)) * self.pixel_difference_factor
        if ldiff < self.args.difference_threshold:
            return False
        
        found,corners = cv2.findChessboardCorners(frame,self.board_dims)
        
        if not found:
            return False
        
        self.camera.current_corners = corners
        
        return True
        
    
    def __automatic_filter_basic_stereo(self):
        l_frame = self.cameras[0].frame
        r_frame = self.cameras[1].frame
        
        lfound,lcorners = cv2.findChessboardCorners(l_frame,self.board_dims)
        rfound,rcorners = cv2.findChessboardCorners(r_frame,self.board_dims)
        if not (lfound and rfound):
            return False
        
        self.cameras[0].current_corners = lcorners
        self.cameras[1].current_corners = rcorners
        
        return True
    
    def __automatic_filter_basic_mono(self):
        frame = self.camera.frame
        found,corners = cv2.findChessboardCorners(frame,self.board_dims)  
        self.camera.current_corners = corners
        return found

    def load_frame_images(self):
        print("Loading frames from '{0:s}'".format(self.full_frame_folder_path))
        files = [f for f in os.listdir(self.full_frame_folder_path) 
                 if osp.isfile(osp.join(self.full_frame_folder_path,f)) and f.endswith(".png")]
        files.sort()
        
        usable_frame_ct = sys.maxint
        
        for camera in self.cameras:
            #assume matching numbers in corresponding left & right files
            files = [f for f in files if f.startswith(self.l_vid_name)]
            cam_frame_ct = 0
            for ix_pair in range(len(files)):
                #TODO: assumes there is the same number of frames for all videos, and all frame
                #indexes match
                frame = cv2.imread(osp.join(self.full_frame_folder_path,files[ix_pair]))
                found,lcorners = cv2.findChessboardCorners(frame,self.board_dims)
                if not found:
                    raise ValueError("Could not find corners in image '{0:s}'".format(files[ix_pair]))
                grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                cv2.cornerSubPix(grey, lcorners, (11,11),(-1,-1),self.criteria_subpix)
                camera.imgpoints.append(lcorners)
                cam_frame_ct+=1
            usable_frame_ct = min(usable_frame_ct,cam_frame_ct)
        for i_frame in range(usable_frame_ct):#@UnusedVariable
            self.objpoints.append(self.board_object_corner_set)
        return usable_frame_ct
    
    def add_corners(self, usable_frame_ct, report_interval, i_frame):
        if(usable_frame_ct % report_interval == 0):
            print ("Usable frames: {0:d} ({1:.3%})"
                   .format(usable_frame_ct, float(usable_frame_ct)/(i_frame+1)))
            
        for camera in self.cameras:
            grey_frame = cv2.cvtColor(camera.frame,cv2.COLOR_BGR2GRAY)
            cv2.cornerSubPix(grey_frame, camera.current_corners, (11,11),(-1,-1),self.criteria_subpix)
            if(self.args.save_images):
                fname = (osp.join(self.full_frame_folder_path,
                               "{0:s}{1:04d}{2:s}".format(camera.name,i_frame,".png")))
                cv2.imwrite(fname, camera.frame)
            camera.imgpoints.append(camera.current_corners)
        self.objpoints.append(self.board_object_corner_set)
    
    def filter_frame_manually(self):
        if len(self.cameras) == 2:
            display_image = np.hstack((self.cameras[0].frame, self.cameras[1].frame))
        else:
            display_image = self.cameras[0].frame
        cv2.imshow("frame", display_image)
        key = cv2.waitKey(0) & 0xFF
        add_corners = (key == ord('a'))
        cv2.destroyWindow("frame")
        return add_corners, key
    
    def run_capture_deterministic_count(self):
        skip_interval = int(self.total_frames / self.args.frame_count_target)

        continue_capture = 1
        for camera in self.cameras:
            #init capture
            camera.read_next_frame()
            continue_capture &= camera.more_frames_remain
            
        usable_frame_ct = 0
        i_start_frame = 0
        i_frame = 0
         
        report_interval = 10
        
        while continue_capture:
            add_corners = False
            i_frame = i_start_frame
            i_end_frame = i_start_frame + skip_interval
            for camera in self.cameras:
                camera.scroll_to_frame(i_frame)
            while not add_corners and i_frame < i_end_frame and continue_capture:
                add_corners = self.__automatic_filter()
                if self.args.manual_filter:
                    add_corners, key = self.filter_frame_manually()
                      
                if add_corners:
                    usable_frame_ct += 1
                    self.add_corners(usable_frame_ct, report_interval, i_frame)
                    #log last usable **filtered** frame
                    for camera in self.cameras:
                        camera.set_previous_to_current()
  
                i_frame += 1
                continue_capture = 1
                for camera in self.cameras:
                    camera.read_next_frame()
                    continue_capture &= camera.more_frames_remain
                continue_capture &= (not (self.args.manual_filter and key == 27))
            i_start_frame = i_end_frame
            
        if self.args.manual_filter and key == 27:
            continue_capture = False
        if self.args.manual_filter:
            cv2.destroyAllWindows()
        return usable_frame_ct
            
    def run_capture(self):
        continue_capture = 1
        for camera in self.cameras:
            #just in case we're running capture again
            camera.scroll_to_beginning()
            #init capture
            camera.read_next_frame()
            continue_capture &= camera.more_frames_remain
        
        report_interval = 10
        i_frame = 0
        usable_frame_ct = 0

        while(continue_capture):
            if not self.args.frame_numbers or i_frame in self.frame_numbers:
                add_corners = self.__automatic_filter()
                
                if self.args.manual_filter:
                    add_corners, key = self.filter_frame_manually()
                      
                if add_corners:
                    usable_frame_ct += 1
                    self.add_corners(usable_frame_ct, report_interval, i_frame)
                    
                    #log last usable **filtered** frame
                    for camera in self.cameras:
                        camera.set_previous_to_current()
      
            i_frame += 1
            continue_capture = 1
            for camera in self.cameras:
                camera.read_next_frame()
                continue_capture &= camera.more_frames_remain
            continue_capture &= (not (self.args.manual_filter and key == 27))
            
        if self.args.manual_filter:
            cv2.destroyAllWindows()
        return usable_frame_ct
        
    def gather_frame_data(self):
        self.objpoints = []
        print("Gathering frame data...")
        usable_frame_ct = 0
        if(self.args.load_corners):
            print("Loading corners from {0:s}".format(self.full_corners_path))    
            imgpoints, self.objpoints, usable_frame_ct =\
            cio.load_corners(self.full_corners_path)
            usable_frame_ct = len(self.objpoints)
            for camera in self.cameras:
                camera.imgpoints = imgpoints[camera.index]
        else:
            if(self.args.load_images):
                usable_frame_ct = self.load_frame_images()
            elif(self.args.frame_count_target != -1):
                usable_frame_ct = self.run_capture_deterministic_count()
            else:
                usable_frame_ct = self.run_capture()            
            if self.args.save_corners:
                print("Saving corners to {0:s}".format(self.full_corners_path))
                file_dict = {}
                for camera in self.cameras:
                    file_dict["imgpoints"+str(camera.index)] = camera.imgpoints
                file_dict["object_point_set"]=self.board_object_corner_set
                np.savez_compressed(self.full_corners_path,**file_dict)
                
        print ("Total usable frames: {0:d} ({1:.3%})"
               .format(usable_frame_ct, float(usable_frame_ct)/self.total_frames))
        self.usable_frame_count = usable_frame_ct
               
    def run_calibration(self):
        min_frames = CalibrateVideoApplication.min_frames_to_calibrate
        if self.usable_frame_count < min_frames:
            print("Not enough usable frames to calibrate."+
                  " Need at least {0:d}, got {1:d}".format(min_frames,self.usable_frame_count))
            return
        print ("Calibrating for max. {0:d} iterations...".format(self.args.max_iterations))
        
        if len(self.cameras) > 1:
            calibration_result = cutils.stereo_calibrate(self.cameras[0].imgpoints, 
                                                         self.cameras[1].imgpoints, 
                                                         self.objpoints, self.frame_dims, 
                                                         self.args.use_fisheye_distortion_model, 
                                                         self.args.use_8_distortion_coefficients, 
                                                         self.args.use_tangential_distortion_coefficients, 
                                                         self.args.precalibrate_solo, 
                                                         self.args.max_iterations, 
                                                         self.path_to_calib_file )
            if self.args.preview:
                l_im = cv2.imread(osp.join(self.args.folder,self.args.preview_files[0]))
                r_im = cv2.imread(osp.join(self.args.folder,self.args.preview_files[1]))
                l_im, r_im = cutils.generate_preview(calibration_result, l_im, r_im)
                path_l = osp.join(self.args.folder,self.args.preview_files[0][:-4] + "_rect.png")
                path_r = osp.join(self.args.folder,self.args.preview_files[1][:-4] + "_rect.png")
                cv2.imwrite(path_l, l_im)
                cv2.imwrite(path_r, r_im)
        else:
            flags = 0
            if self.path_to_calib_file != None:
                result = cio.load_opencv_stereo_calibration(self.path_to_calib_file)
                if(self.frame_dims != result.resolution):
                    raise ValueError("Resolution in specified calibration file (" + 
                                     self.path_to_calib_file + ") does not correspond to given resolution.")
                flags += cv2.CALIB_USE_INTRINSIC_GUESS
            criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, self.args.max_iterations, 
                        2.2204460492503131e-16)
            if not self.args.use_tangential_distortion_coefficients:
                flags += cv2.CALIB_ZERO_TANGENT_DIST
            if self.args.use_8_distortion_coefficients:
                flags += cv2.CALIB_RATIONAL_MODEL
            calibration_result = cutils.calibrate(self.objpoints, self.camera.imgpoints, flags, 
                                                  criteria, self.camera.calib)
        if not self.args.skip_printing_output:
            print(calibration_result)
        if not self.args.skip_saving_output:
            cio.save_opencv_stereo_calibration(osp.join(self.args.folder,self.args.output), calibration_result)

class Setting(Enum):
    folder = "folder"
    frame_numbers = "frame_numbers"
    videos = "videos"
    preview_files = "preview_files"
    preview = "preview"
    board_width = "board_width"
    board_height = "board_height"
    board_square_size = "board_square_size"
    sharpness_threshold = "sharpness_threshold"
    difference_threshold = "difference_threshold"
    manual_filter = "manual_filter"
    frame_count_target = "frame_count_target"
    corners_file = "corners_file"
    save_corners = "save_corners"
    load_corners = "load_corners"
    settings_file = "settings_file"
    max_iterations = "max_iterations"
    precalibrate_solo = "precalibrate_solo"
    use_8_distortion_coefficients = "use_8_distortion_coefficients"
    use_tangential_distortion_coefficients = "use_tangential_distortion_coefficients"
    use_fisheye_distortion_model = "use_fisheye_distortion_model"
    skip_printing_output = "skip_printing_output"
    skip_saving_output = "skip_saving_output"
    output = "output"
    use_existing = "use_existing"
    filtered_image_folder = "filtered_image_folder"
    save_images = "save_images"
    load_images = "load_images"

def required_length(nmin,nmax):
    class RequiredLength(ap.Action):
        def __call__(self, conf_parser, args, values, option_string=None):
            if not nmin<=len(values)<=nmax:
                msg='argument "{f}" requires between {nmin} and {nmax} arguments'.format(
                    f=self.dest,nmin=nmin,nmax=nmax)
                raise ap.ArgumentTypeError(msg)
            setattr(args, self.dest, values)
    return RequiredLength

def main(argv=None):
    defaults = {
        Setting.folder:"./",
        Setting.frame_numbers:None,
        Setting.videos: ["left.mp4","right.mp4"],
        Setting.preview_files:["left.png","right.png"],
        Setting.preview:False,
        Setting.board_width:9,
        Setting.board_height:6,
        Setting.board_square_size:1.98888,
        Setting.sharpness_threshold:55,
        Setting.difference_threshold:0.4,
        Setting.manual_filter:False,
        Setting.frame_count_target:-1,
        Setting.corners_file:"corners.npz",
        Setting.save_corners:False,
        Setting.load_corners:False,
        Setting.settings_file:None,
        Setting.max_iterations:30,
        Setting.precalibrate_solo:False,
        Setting.use_8_distortion_coefficients:False,
        Setting.use_tangential_distortion_coefficients:False,
        Setting.use_fisheye_distortion_model:False,
        Setting.skip_printing_output:False,
        Setting.skip_saving_output:False,
        Setting.output:None,
        Setting.use_existing:False,
        Setting.filtered_image_folder:"frames",
        Setting.save_images:False,
        Setting.load_images:False
        }
    
    #============== STORAGE/RETRIEVAL OF SETTINGS =====================================================#
    conf_parser = ap.ArgumentParser(description='Traverse two .mp4 stereo video files and '+
                               ' stereo_calibrate the cameras based on specially selected frames within.',
                               formatter_class=ap.RawDescriptionHelpFormatter, add_help=False)
    conf_parser.add_argument("-sf", "--" + Setting.settings_file.name, required = False, 
                        default=defaults[Setting.settings_file], 
                        help="File to save to / load from settings for the program.")
    
    args = conf_parser.parse_known_args()
    if(args.conf_file):
        config = ConfigParser.SafeConfigParser(defaults)
        config.read([args.settings_file])
        defaults.update(dict(config.items("Defaults")))
    
    parser = ap.ArgumentParser(parents=[conf_parser])
    parser.add_argument("-f", "--" + Setting.folder.name, help="Path to root folder to work in", 
                        required=False, default=defaults[Setting.folder])
    parser.add_argument("-fn", "--" + Setting.frame_numbers.name, help="frame numbers .npz file with"+
                        " frame_numbers array."+
                        " If specified, program filters frame pairs based on these numbers instead of other"+
                        " criteria.",required=False, default=None)
    parser.add_argument("-v", "--" + Setting.videos.name, nargs='+', action=required_length(1, 2),
                        help="input stereo video tuple (left, right)", 
                        required=False, default=defaults[Setting.videos])
    
    #============== CALIBRATION PREVIEW ===============================================================#
    #currently does not work due to OpenCV python bindings bug
    parser.add_argument("-pf", "--" + Setting.preview_files.name, nargs='+', help="input frames to test"+
                        " calibration result (currently only for stereo)", 
                        required=False, default= ["left.png","right.png"], action=required_length(1, 2))
    parser.add_argument("-p", "--" + Setting.preview.name, help="Test calibration result on left/right"+
                        " frame pair (currently only for stereo)", 
                        action = "store_true", required=False, default=defaults[Setting.preview])
    
    #============== BOARD DIMENSIONS ==================================================================#
    parser.add_argument("-bw", "--" + Setting.board_width.name, help="checkerboard inner corner width",
                        required = False, default=defaults[Setting.board_width], type=int)
    parser.add_argument("-bh", "--" + Setting.board_height.name, help="checkerboard inner corner height",
                        required = False,default=defaults[Setting.board_height], type=int)
    parser.add_argument("-bs", "--" + Setting.board_square_size.name, help="checkerboard square size, in meters", 
                        required = False, type=float, default=defaults[Setting.board_square_size])
    
    #============== FRAME FILTERING CONTROLS ==========================================================#
    parser.add_argument("-ft", "--" + Setting.sharpness_threshold.name, 
                        help="sharpness threshold based on variance of "+
                        "Laplacian; used to filter out frames that are too blurry (default 55.0).", 
                        type=float, required = False, default=defaults[Setting.sharpness_threshold])
    parser.add_argument("-fd", "--" + Setting.difference_threshold.name, 
                        help="difference threshold: minimum average "
                        +" per-pixel difference (in range [0,1.0]) between current and previous frames to "
                        +"filter out frames that are too much alike (default: 0.4)", type=float, 
                        required = False, default=defaults[Setting.difference_threshold])
    parser.add_argument("-m", "--" + Setting.manual_filter.name, 
                        help="pick which (pre-filtered)frames to use manually"+
                        " one-by-one (use 'a' key to approve)", required = False, action='store_true', 
                        default=defaults[Setting.manual_filter])
    parser.add_argument("-fc", "--" + Setting.frame_count_target.name, required=False, 
                        default=defaults[Setting.frame_count_target], type=int,
                        help="total number of frames (from either camera) to target for calibration.")
    
    #============== STORAGE OF BOARD CORNER POSITIONS =================================================#
    parser.add_argument("-c", "--" + Setting.corners_file.name, required = False, 
                        default=defaults[Setting.corners_file], help="store filtered corners")
    parser.add_argument("-s", "--" + Setting.save_corners.name, action='store_true', required = False, 
                        default=defaults[Setting.save_corners])
    parser.add_argument("-l", "--" + Setting.load_corners.name, action='store_true', required = False, 
                        default=defaults[Setting.load_corners])

    
    #============== CALIBRATION & DISTORTION MODEL CONTROLS ===========================================#
    parser.add_argument("-i", "--" + Setting.max_iterations.name, 
                        help="maximum number of iterations for the stereo"+
                        " calibration (optimization) loop", type=int, required = False, 
                        default=defaults[Setting.max_iterations])
    parser.add_argument("-ds", "--" + Setting.precalibrate_solo.name, help="pre-stereo_calibrate each camera "+
                        "individually (in case of stereo calibration) "+
                        "first, then perform stereo calibration",action='store_true', required = False, 
                        default=defaults[Setting.precalibrate_solo])
    parser.add_argument("-d8", "--" + Setting.use_8_distortion_coefficients.name, action='store_true', required = False, 
                        default=defaults[Setting.use_8_distortion_coefficients])
    parser.add_argument("-dt", "--" + Setting.use_tangential_distortion_coefficients.name, action='store_true', 
                        required = False, default=defaults[Setting.use_tangential_distortion_coefficients])
    parser.add_argument("-df", "--" + Setting.use_fisheye_distortion_model.name, action='store_true', 
                        required = False, default=defaults[Setting.use_fisheye_distortion_model])
    
    parser.add_argument("-skp", "--" + Setting.skip_printing_output.name, action='store_true', 
                        required = False, default= defaults[Setting.skip_printing_output])
    parser.add_argument("-sks", "--" + Setting.skip_saving_output.name, action='store_true', 
                        required = False, default= defaults[Setting.skip_saving_output])
    
    parser.add_argument("-o", "--" + Setting.output.name, help="output file to store calibration results", 
                        required = False, default=defaults[Setting.output])
    #TODO this should be a separate setting from output file
    parser.add_argument("-u", "--" + Setting.use_existing.name, 
                        help="use the existing output file to initialize calibration parameters", 
                        action="store_true", required = False, default=defaults[Setting.use_existing])
    
    #============== FILTERED IMAGE/FRAME BACKUP & LOADING =============================================#
    parser.add_argument("-if", "--" + Setting.filtered_image_folder.name, help="filtered frames"+
                        " will be saved into this folder (relative to work folder specified by --folder)", 
                        required = False, default=defaults[Setting.filtered_image_folder])
    parser.add_argument("-is", "--" + Setting.save_images.name, action='store_true', required = False, 
                        default= defaults[Setting.save_images])
    parser.add_argument("-il", "--" + Setting.load_images.name, action='store_true', required = False, 
                        default= defaults[Setting.load_images])
   

    
    
    app = CalibrateVideoApplication(args)
    app.gather_frame_data()
    app.run_calibration()
    
if __name__ == "__main__":
    sys.exit(main())
        
    
