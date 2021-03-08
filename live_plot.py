import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
import matplotlib.gridspec as gridspec
import time
import cv2
from collections import deque
import numpy as np
import argparse

parser = argparse.ArgumentParser(description='Live Plot Graphs.')
				   
fig = plt.figure(dpi = 100, figsize=(12, 8))
gs = gridspec.GridSpec(ncols=4, nrows=6)
ax1 = fig.add_subplot(gs[0:4, 0:2])
ax2 = fig.add_subplot(gs[0:3, 2:4])
ax3 = fig.add_subplot(gs[3:6, 2:4])
ax4 = fig.add_subplot(gs[4:6, 0:2])


left  = 0.125  # the left side of the subplots of the figure
right = 0.9    # the right side of the subplots of the figure
bottom = 0.1   # the bottom of the subplots of the figure
top = 0.9      # the top of the subplots of the figure
wspace = 0.6   # the amount of width reserved for blank space between subplots
hspace = 0.8   # the amount of height reserved for white space between subplots
fig.subplots_adjust(left=None, bottom=None, right=None, top=top, wspace=wspace, hspace=hspace)

def animate(i):
	# subplot 1
	img = cv2.imread("/home/imacs/Desktop/imacs/out_imgs/img_binary.png")
	if img is not None:
		ax1.clear()
		#ax3.set_xlabel('time (sec)', fontsize=15)
		ax1.set_title("Sliding Window Based Lane Tracking", fontsize=18)
		#ax3.set_ylabel('lateral deviation (cm)', fontsize=15)
		ax1.grid(alpha=0.3, linestyle='-.', zorder=-3)
		#ax[i].legend(["VER:{}".format(0), "VER:{}".format(i)], loc='upper right', frameon=False, prop={'size': 12}, bbox_to_anchor=(1.01, 1.01))
		#ax1.imshow(img)
		ax1.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
	else:
		print("img_binary is None")
	
	# subplot 2
	img = cv2.imread("/home/imacs/Desktop/imacs/out_imgs/img_output.png")
	if img is not None:
		ax2.clear()
		#ax4.set_xlabel('time (sec)', fontsize=15)
		#ax4.set_ylabel('lateral deviation (cm)', fontsize=15)
		ax2.grid(alpha=0.3, linestyle='-.', zorder=-3)
		ax2.set_title("Final Output Image", fontsize=18)
		#ax[i].legend(["VER:{}".format(0), "VER:{}".format(i)], loc='upper right', frameon=False, prop={'size': 12}, bbox_to_anchor=(1.01, 1.01))
		#ax2.imshow(img)
		ax2.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
	else:
		print("img_output is None")
	
	# subplot 3
	pullData = open("/home/imacs/Desktop/imacs/csv/results/results_actual_yL.csv","r").read()
	dataArray = pullData.split('\n')
	xaref1 = []
	yaref1 = []
	saref1 = []
	for eachLine in dataArray[26:]:
		if len(eachLine)>1:
			x,y = eachLine.split(',')
			xaref1.append(float(x))
			yaref1.append(float(y)*100)
			#saref1.append(float(m))	
	ax3.clear()
	ax3.set_xlabel('time (sec)', fontsize=15)
	ax3.set_ylabel('lateral deviation (cm)', fontsize=15)
	ax3.grid(alpha=0.3, linestyle='-.', zorder=-3)
	ax3.plot(xaref1,yaref1, marker='.',  markersize=4, alpha=0.6, zorder=-1, linestyle='-', linewidth = 0.5)
	#ax3.set_ylim([-15,15]) 
	# subplot 4
	pullDatadf = open("/home/imacs/Desktop/imacs/csv/results/results_steering_angle.csv","r").read()
	dataArray2 = pullDatadf.split('\n')
	xaref2 = []
	yaref2 = []
	saref2 = []
	for eachLine in dataArray2[6:]:
		if len(eachLine)>1:
			x,y = eachLine.split(',')
			xaref2.append(float(x))
			yaref2.append(float(y))
			#saref1.append(float(m))	
	ax4.clear()
	ax4.set_xlabel('time (sec)', fontsize=15)
	ax4.set_ylabel('steering angle (radians)', fontsize=15)
	ax4.grid(alpha=0.3, linestyle='-.', zorder=-3)
	ax4.plot(xaref2,yaref2, marker='.',  markersize=4, alpha=0.6, zorder=-1, linestyle='-', linewidth = 0.5)
	#ax4.set_ylim([-15,15])


ani = animation.FuncAnimation(fig, animate, interval=1000)
# Keep a reference to the FuncAnimation, so it does not get garbage collected
# animation = animation.FuncAnimation(fig, updateGraph, frames=200,
                    # init_func=initGraph,  interval=250, blit=True)

plt.show()

