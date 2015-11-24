#!/usr/bin/python

'''
Created on Nov 24, 2015

@author: algomorph
'''

import calib as cb
import argparse
import cv2
import os.path as osp

parser = argparse.ArgumentParser(description='Rectify a stereo image pair using existing calibration.')
parser.add_argument("-f", "--folder", help="Folder to work in",
                    required=False, default= "./")
parser.add_argument("-c", "--calibration", help="Calibration file", 
                    required=False, default= "calib.xml")
parser.add_argument("-i", "--images", nargs=2, help="Input files (left, right) to test calibration result on.", 
                    required=False, default= ["left.png","right.png"])
parser.add_argument("-o", "--output", nargs=2, help="Output files (left, right).", 
                    required=False, default= ["rect_left.png","rect_right.png"])

if __name__ == '__main__':
    args = parser.parse_args()
    error, K1, d1, K2, d2, R, T, prev_dims = cb.load_opencv_calibration(osp.join(args.folder,args.calibration))
    test_im_left = cv2.imread(osp.join(args.folder,args.images[0]))
    test_im_right = cv2.imread(osp.join(args.folder,args.images[1]))
    rect_left, rect_right = cb.generate_preview(K1, d1, K2, d2, R, T, test_im_left, test_im_right)
    cv2.imwrite(osp.join(args.folder,args.output[0]), rect_left)
    cv2.imwrite(osp.join(args.folder,args.output[1]), rect_right)
    
    