#!/usr/bin/python3

import sys
import argparse as ap
from enum import Enum
import configparser
from calib.app import CalibrateVideoApplication


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
    
    args, remaining_argv = conf_parser.parse_known_args()
    if(args.conf_file):
        config = configparser.SafeConfigParser(defaults)
        config.read([args.settings_file])
        defaults.update(dict(config.items("Defaults")))
    
    parser = ap.ArgumentParser(parents=[conf_parser])
    parser.set_defaults(**defaults)
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
   
    args = parser.parse_args(remaining_argv)
    app = CalibrateVideoApplication(args)
    app.gather_frame_data()
    app.run_calibration()
    
if __name__ == "__main__":
    sys.exit(main())
        
    
