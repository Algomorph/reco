#!/usr/bin/python

import cv2
import os.path as osp
import argparse as ap

conf_parser = ap.ArgumentParser(description='A simple utility that overlays one image over another.')

conf_parser.add_argument("-f", "--folder", help="Folder to work in", 
                    required=False, default= "./")
conf_parser.add_argument("-i", "--images", nargs=2, help="Names of two image files", 
                    required=False, default= ["left.png","right.png"])
conf_parser.add_argument("-w", "--weights", nargs=2, help="Weights to use for overlay (must add up to one)", 
                    required=False, type=float, default= [0.5,0.5])
conf_parser.add_argument("-o", "--output", help="name of the output file", 
                    required=False, default= "overlay.png")


if __name__ == "__main__":
    args = conf_parser.parse_args()
    
    left = cv2.imread(osp.join(args.folder,args.images[0]))
    
    right = cv2.imread(osp.join(args.folder,args.images[1]))
    
    combined = cv2.addWeighted(left,args.weights[0],right,args.weights[1],0)
      
    cv2.imwrite(osp.join(args.folder,args.output),combined);