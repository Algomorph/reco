#!/usr/bin/python

import cv2
import argparse as ap

parser = ap.ArgumentParser(description='Check whether epipolar lines in two stereo images are horizontal and coincide')
parser.add_argument("-i", "--images", nargs=2, help="Input stereo image tuple (left, right)", 
                    required=False, default= ["left.png","right.png"])

if __name__ == "__main__":
    args = parser.parse_args()
    
    left = cv2.imread(args.images[0])
    
    right = cv2.imread(args.images[1])
    
    combined = cv2.addWeighted(left,0.5,right,0.5,0)
    
    colors = [(168,168,20),(20,255,240),(200,40,20)]
    
    i_line = 0
    for y_line in xrange(0,combined.shape[0],20):
        cv2.line(combined, (0,y_line),(combined.shape[1]-1,y_line),colors[i_line%3])
        i_line +=1
    
    cv2.imwrite("epipolar_lines.png",combined);