#!/usr/bin/python

import os
import os.path as osp
import argparse as ap
import numpy as np
import screeninfo as scr
import re
import cv2


conf_parser = ap.ArgumentParser(description='Delete all right frame files in folder based on which left frame files remain.')

conf_parser.add_argument("-f", "--folder", help="Folder to work in", 
                    required=False, default= ["./"])

if __name__ == "__main__":
    args = conf_parser.parse_args()
    files = [f for f in os.listdir(args.folder) if osp.isfile(osp.join(args.folder,f)) and f.endswith(".png")]
    files.sort()
    
    rfiles = [f for f in files if "r_" in f]
    lfiles = [f for f in files if "l_" in f]
    rfiles.sort()
    lfiles.sort()
    
    monitors = scr.get_monitors()
    main_monitor = monitors[0]
    screen_height = main_monitor.height
    header_height = 100
    resized_height = 250
    padding = 10
    
    #assume frames of equal dimensions for right & left
    sample_frame = cv2.imread(osp.join(args.folder,lfiles[0]))
    resized_width = int(sample_frame.shape[1] * resized_height / sample_frame.shape[0])
    new_size = (resized_width,resized_height)
    step = int((screen_height - header_height - padding) / (resized_height+padding))
    pane_width = 2*resized_width + (3*padding)
    pane_height = screen_height - header_height
    pane_dims = (pane_height,pane_width,3)
    x_left = padding
    x_right = padding+resized_width+padding
    font = cv2.FONT_HERSHEY_PLAIN
    for first_in_page in xrange(0,len(rfiles),step):
        pane = np.zeros(pane_dims,np.uint8)
        vertical_pos = padding
        max_i = min((step,len(rfiles)-first_in_page))
        for i_in_page in xrange(0, step):
            lname = lfiles[first_in_page + i_in_page]
            rname = rfiles[first_in_page + i_in_page]
            lim = cv2.imread(osp.join(args.folder,lname))
            rim = cv2.imread(osp.join(args.folder,rname))
            lim_resized = cv2.resize(lim, new_size)
            rim_resized = cv2.resize(rim, new_size)
            cv2.putText(lim_resized, lname, (200,20),font,1,(20,110,0),2)
            cv2.putText(rim_resized, rname, (200,20),font,1,(20,110,0),2)
            pane[vertical_pos:vertical_pos+resized_height, x_left:x_left+resized_width,:]=lim_resized
            pane[vertical_pos:vertical_pos+resized_height, x_right:x_right+resized_width,:]=rim_resized
            vertical_pos += (resized_height + padding)
        cv2.imshow("Pane", pane)
        key = cv2.waitKey() & 0xFF
        if(key == 27):
            break