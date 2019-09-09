import numpy as np
from numpy.linalg import multi_dot
import math
import random


class ground:
	def __init__(self, boundX, boundY):
		self.xmin = -1*boundX/2
		self.ymin = -1*boundY/2
		self.xmax = boundX/2
		self.ymax = boundY/2
		self.X = np.arange(self.xmin, self.xmax, 1)
		self.Y = np.arange(self.ymin, self.ymax, 1)
		self.Z = []
		self.corner1 = []
		self.corner2 = []
		self.corner3 = []
		self.corner4 = []
		self.create()


	def create(self):
		self.X, self.Y = np.meshgrid(self.X, self.Y)
		size = self.X.shape
		self.Z = np.zeros(size)
		self.corner1 = [self.xmin, self.ymin, 0]
		self.corner2 = [self.xmin, self.ymax, 0]
		self.corner3 = [self.xmax, self.ymax, 0]
		self.corner4 = [self.xmax, self.ymin, 0]

class xy_plane:
	def __init__(self, boundX, boundY, X_pos = 0, Y_pos = 0, Z_pos = 0):
		self.xmin = -1*boundX/2
		self.ymin = -1*boundY/2
		self.xmax = boundX/2
		self.ymax = boundY/2
		self.X = np.arange(self.xmin, self.xmax, 1) + X_pos
		self.Y = np.arange(self.ymin, self.ymax, 1) + Y_pos
		self.Z = Z_pos
		self.create()


	def create(self):
		self.X, self.Y = np.meshgrid(self.X, self.Y)
		size = self.X.shape
		self.Z = self.Z + np.zeros(size)

class xz_plane:
	def __init__(self, boundX, boundZ, X_pos = 0, Y_pos = 0, Z_pos = 0):
		self.xmin = -1*boundX/2
		self.zmin = 0
		self.xmax = boundX/2
		self.zmax = boundZ
		self.X = np.arange(self.xmin, self.xmax, 1) + X_pos
		self.Y = Y_pos
		self.Z = np.arange(self.zmin, self.zmax, 1) + Z_pos
		self.create()


	def create(self):
		self.X, self.Z = np.meshgrid(self.X, self.Z)
		size = self.X.shape
		self.Y = self.Y + np.zeros(size)

class yz_plane:
	def __init__(self, boundY, boundZ, X_pos = 0, Y_pos = 0, Z_pos = 0):
		self.ymin = -1*boundY/2
		self.zmin = 0
		self.ymax = boundY/2
		self.zmax = boundZ
		self.X = X_pos
		self.Y = np.arange(self.ymin, self.ymax, 1) + Y_pos
		self.Z = np.arange(self.zmin, self.zmax, 1) + Z_pos
		self.create()


	def create(self):
		self.Y, self.Z = np.meshgrid(self.Y, self.Z)
		size = self.Y.shape
		self.X = self.X + np.zeros(size)

class plane:
	def __init__(self, corner1, corner2, corner3, corner4): #takes two diagonal points
		self.corner1 = corner1
		self.corner2 = corner2
		self.corner3 = corner3
		self.corner4 = corner4
		dimensions = [0, 1, 2]
		excluded_dimension = -1
		coordinates = []
		for index, value in enumerate(corner1):
			if value == corner2[index]:
				coordinates.append(value)
				excluded_dimension = index
				continue
			if value > corner2[index]:
				grid = np.arange(corner2[index], value, 1)
			else:
				grid = np.arange(value, corner2[index], 1)
			coordinates.append(grid)

		dimensions.remove(excluded_dimension)
		coordinates[dimensions[0]], coordinates[dimensions[1]] = np.meshgrid(coordinates[dimensions[0]], coordinates[dimensions[1]])
		size = coordinates[dimensions[0]].shape
		coordinates[excluded_dimension] = coordinates[excluded_dimension] + np.zeros(size)
		self.X = coordinates[0]
		self.Y = coordinates[1]
		self.Z = coordinates[2]

def box(point1, point2, point3, point4):
	ground1 = (point1[0], point1[1], 0)
	ground2 = (point2[0], point2[1], 0)
	ground3 = (point3[0], point3[1], 0)
	ground4 = (point4[0], point4[1], 0)
	plane1 = plane(ground1, point2, point1, ground2) 
	plane2 = plane(ground2, point3, point2, ground3)
	plane3 = plane(ground3, point4, point3, ground4)
	plane4 = plane(ground4, point1, point4, ground1)
	plane5 = plane(point1, point3, point2, point4)
	return (plane1, plane2, plane3, plane4, plane5)

		
def cube(centerX, centerY, side):
	center1 = (centerX, centerY-side/2, 0)
	center2 = (centerX-side/2, centerY, 0)
	center3 = (centerX, centerY+side/2, 0)
	center4 = (centerX+side/2, centerY, 0)
	plane1 = xz_plane(side,side,*center1) 
	plane2 = yz_plane(side,side,*center2)
	plane3 = xz_plane(side,side,*center3)
	plane4 = yz_plane(side,side,*center4)
	plane5 = xy_plane(side,side,centerX,centerY,side)
	return (plane1, plane2, plane3, plane4, plane5)


# class box:
# 	def __init__(self, corner1, corner2, corner3, corner4):
# 		# get x,y,z coordinates from corner tuples
# 		# make plane objects based on corner tuples
# 		self.create()


# 	def create(self):
