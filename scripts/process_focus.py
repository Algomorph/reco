#!/usr/bin/python3

import cv2; import numpy as np; import matplotlib.pyplot as plt
import os.path as osp; import os; import re


if __name__ == "__main__":
	file_names = os.listdir('.')
	file_names.sort()
	#get all the png image file names
	needed_file_names = np.array([fn for fn in file_names if not re.match(r'-?\d*\.?\d+\.png$',fn) is None])
	image_numbers = np.array([float(fn[:-4]) for fn in needed_file_names])
	#sort according to numerical values
	order = np.argsort(image_numbers)
	image_numbers = image_numbers[order]
	needed_file_names = needed_file_names[order]
	images = [cv2.imread(fn) for fn in needed_file_names]
	variances = [np.var(cv2.Laplacian(img,cv2.CV_64F)) for img in images]
	fig = plt.figure()
	plt.plot(image_numbers,variances,'bo-')
	fig.savefig("sharpness_plot.png")
	np.savetxt("sharpness_data.txt",variances)



	
