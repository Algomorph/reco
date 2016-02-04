#!/usr/bin/python3
import argparse
import numpy as np
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description='Create a plot from serveral series.')
parser.add_argument("-s","--series", nargs='+',help='series file paths', required=True)
parser.add_argument("-ds","--domain_start", type=int,help='start value on the x axis', required=True)
parser.add_argument("-de","--domain_end", type=int,help='end value on the x axis', required=True)

if __name__ == "__main__":
	args = parser.parse_args()
	series = []
	x_series = np.arange(args.domain_start,args.domain_end)
	colors = ['b','g','r','c','m','k']
	plot_args = []
	ix = 0
	for path in args.series:
		cur_series = np.loadtxt(path)
		series.append(cur_series)
		plot_args.append(x_series)
		plot_args.append(cur_series[0:len(x_series)])
		plot_args.append(colors[ix % len(colors)])
		ix += 1
	
	fig = plt.figure()
	plt.plot(*plot_args)
	fig.savefig("sharpness_comparison_plot.png")


