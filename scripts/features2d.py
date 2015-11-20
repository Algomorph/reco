#!/usr/bin/python

import cv2
import cv2.xfeatures2d as x2d
import numpy as np
import argparse as ap
import os
import os.path as osp


parser = ap.ArgumentParser(description='Detect & match 2d features in 2 images.')
parser.add_argument("-f", "--folder", help="Folder to work in", 
                    required=False, default= ["./"])
parser.add_argument("-fn", "--filenames", help="Names of the left and right images",
                    required=False, nargs=2, default = ["left.png","right.png"])

if __name__ == "__main__":
    args = parser.parse_args()
    im_l = cv2.imread(osp.join(args.folder,args.filenames[0]))
    im_r = cv2.imread(osp.join(args.folder,args.filenames[1]))
    
    sift = x2d.SIFT_create(1000)
    features_l, des_l = sift.detectAndCompute(im_l, None)
    features_r, des_r = sift.detectAndCompute(im_r, None)
    
    #flann currently seems broken due to bug in opencv
#     FLANN_INDEX_KDTREE = 1
#     index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
#     search_params = dict(checks=50)
#     flann = cv2.FlannBasedMatcher(index_params,search_params)
#     
#     matches = flann.knnMatch(des_l, des_r, k=2)