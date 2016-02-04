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
    
    