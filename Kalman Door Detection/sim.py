#! /usr/bin/python

import kf
from scipy import stats
import scipy.stats
import numpy as np
import math
import cv2
import random

def drawDoors(img, doors):
	for pos in doors:
		cv2.rectangle(img, (100 * pos + 10,0), (100 * pos + 90, 65), (39, 142, 229), -1)

	cv2.line(img,(0,70), (1000,70), (255,255,255), 3)

def drawRobot(img, x):
	cv2.rectangle(img, (x-15,60), (x+15,20), (255,0,0),-1)

	cv2.ellipse(img,(x,20),(10,12),0,180,359,(0,255,0),-1)

	cv2.circle(img, ( x-7, 62), 8, (0,0,255), -1)

	cv2.circle(img, ( x+7, 62), 8, (0,0,255), -1)

def drawSensor(img, x):
	cv2.line(img, (int(x),80), (int(x), 200), (0,255,255), 2)

def drawDoorDist(img,dist):
	for i in range(0,10):
		cv2.rectangle(img, (100*i+25,190),(100*i+75, 190-int(90*dist[i])), (0,0,255), -1)

def drawProbDist(img, mean, sigma, color):
	range_min = max(int(mean-4*sigma),1)

	range_max = min(int(mean+4*sigma),1000-1)

	if range_max < range_min:
		return

	pts = np.zeros([range_max-range_min,2], np.int32)

	for x in range(range_min,range_max) :
		pts[x-range_min-1,0] = x

		pts[x-range_min-1,1] = int(200 - scipy.stats.norm.pdf(x, mean, sigma)*1000)

	cv2.polylines(img,[pts],True,color)

def drawDoorSensor(img,x):
	cv2.circle(img, (x, 7), 5, (255,0,255), -1)


def main():

	# locations of doors (todo, read from file/cmd line)
	doors = [2, 4, 9]

	# initial setup commands
	k = 0

	# initial setup
	x = 500

	vel = 0

	xmin = 15

	xmax = 1000-15

	mean = 500

	u = np.matrix([[500.0, 0.0]]).T

	sigma = np.diag([200.0, 50.0])

	door_dist = [0.5] * 10

	random.seed()

	while k != 27:
		# Create a black image
		img = np.zeros((200,xmax + 30,3), np.uint8)

		# draw environment
		drawDoors(img,doors)

		drawProbDist(img, u.item((0,0)), sigma.item((0,0)), (0,255,0))

		# draw robot 
		drawRobot(img, x)

		# draw sensor measurement
		z = np.random.normal(x, 5, 3)

		drawProbDist(img, int(z[0]), 10, (0,255,255))

		drawDoorDist(img, door_dist)

		# simulate measurement
		door_pos = int(x / 100)

		door_meas = False

		num = random.random()

		if door_pos in doors:
			if num <= 0.6:
				door_meas = True

		else:
			if num > 0.8:
				door_meas = True

		if door_meas:
			drawDoorSensor(img,x)

		# show sim
		cv2.namedWindow('image', cv2.WINDOW_AUTOSIZE)

		cv2.imshow('image',img)


		door_dist = kf.door_update(u, sigma, door_meas, door_dist)


		# arrow keys for control (left, right, space to stop, Esc to quit)
		k = cv2.waitKey(30) & 0xff

		if k == 81:
			vel -= 1

		elif k == 83:
			vel += 1

		elif k == 32:
			vel = 0

		# add velocity noise
		vel_rnd = vel

		if vel > 0:
			vel_rnd += (vel+1)*int(np.random.normal(0, 1, 1000)[25])

		x += vel_rnd

		if x > xmax:
			vel = 0

			x = xmax

		if x < xmin:
			vel = 0

			x = xmin

		# add current velocity (intended) to state matrix
		u[1] = vel_rnd + (vel+1)*int(np.random.normal(0, 1, 1000)[25])


		# do kf update
		[u, sigma] = kf.kf_update(u, sigma, z[0])

	cv2.destroyAllWindows()

main()
