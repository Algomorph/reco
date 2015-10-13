#!/usr/bin/python
import os
import sys
import os.path as osp
import cv2
import numpy as np
import argparse as ap

parser = ap.ArgumentParser(description='Traverse all .mp4 video files in the specified folder and pick out frames to export as images.')
parser.add_argument("-f", "--folder", help="Folder to process", 
                    required=False, default= ["./"])

if __name__ == "__main__":
	args = parser.parse_args()
	files = [f for f in os.listdir(args.folder) if osp.isfile(args.folder + f) and f.endswith(".mp4")]
	
	files.sort();
	
	left_right_prefix = True
	left_files = []
	right_files = []
	for file in files:
		if file.endswith("left.mp4"):
			left_files.append(file)
		elif file.endswith("right.mp4"):
			right_files.append(file)
		else:
			left_right_prefix = False
	
	if left_right_prefix:
		num_pairs = len(left_files)
		if len(left_files) != len(right_files):
			print "Warning: there are unequal amounts of files marked as right and files marked as left."
			num_pairs = min(len(left_files), len(right_files))
		for i_pair in range(num_pairs):
			print "Processing pair " + f
			left_file = left_files[i_pair]
			right_file = right_files[i_pair]
			left_cap = cv2.VideoCapture(args.folder + left_file)
			right_cap = cv2.VideoCapture(args.folder + right_file)
			cont_cap = True
			i_frame = 0
			while(cont_cap):
				lret, lframe = left_cap.read()
				rret, rframe = right_cap.read()
				cont_cap = lret and rret
				if cont_cap:
					combo = np.hstack((lframe,rframe))
					cv2.imshow("video",combo)
					key = cv2.waitKey() & 0xFF
					if (key == ord('q') or key == ord('n') or key == 28):
						break
					elif (key == ord('s')):
						fnl = "frame_" + left_file[:-4] + "_" + str(i_frame) + ".png"
						fnr = "frame_" + right_file[:-4] + "_" + str(i_frame) + ".png"
						cv2.imwrite(args.folder + fnl,lframe)
						cv2.imwrite(args.folder + fnr,rframe)
						print "saving frame " + str(i_frame)
					else:
						print "next frame pair"
					i_frame += 1
			left_cap.release()
			right_cap.release()
			if(key == ord('q')):
				break
	else:
		for f in files:
			cap = cv2.VideoCapture(args.folder + f)
			ret = True;
			i_fr = 0;
			key = ''
			print "Processing file " + f
			while(ret):
				ret, frame = cap.read()
				if ret:
					cv2.imshow("video",frame)
					key = cv2.waitKey() & 0xFF
					if (key == ord('q') or key == ord('n')):
						break
					elif (key == ord('s')):
						fn = f[:-4] + "_" + str(i_fr) + ".png"
						cv2.imwrite(args.folder + fn,frame)
						print "saving frame as " + fn
					else:
						print "next frame"
					i_fr += 1
			cap.release()
			if(key == ord('q')):
				break
	
