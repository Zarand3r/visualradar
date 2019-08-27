import numpy as np
from numpy.linalg import multi_dot
import math
import random


class surface:
	def __init__(self, boundX, boundY):
		self.xmin = -1*boundX/2
		self.ymin = -1*boundY/2
		self.xmax = boundX/2
		self.ymax = boundY/2
		self.X = np.arange(self.xmin, self.xmax, 1)
		self.Y = np.arange(self.ymin, self.ymax, 1)
		self.Z = []
		self.create()


	def create(self):
		self.X, self.Y = np.meshgrid(self.X, self.Y)
		size = self.X.shape
		self.Z = np.zeros(size)
		print(self.Z.shape)

# class surface:
# 	def __init__(self, boundX, boundY, func):
# 		self.xmin = 0
# 		self.ymin = 0
# 		self.xmax = boundX
# 		self.ymax = boundY
# 		self.X = np.arange(self.xmin, self.xmax, 1)
# 		self.Y = np.arange(self.ymin, self.ymax, 1)
# 		self.Z = []
#		#	self.Z = func(self.X, self.Y)
# 		self.create()


# 	def create(self):
# 		self.X, self.Y = np.meshgrid(self.X, self.Y)
# 		size = self.X.shape
# 		self.Z = np.zeros(size)

# boundx is length in x dimension, boundy is length in y dimension, boundz is length in z dimension
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


# class box:
# 	def __init__(self, corner1, corner2, corner3, corner4):
# 		# get x,y,z coordinates from corner tuples
# 		# make plane objects based on corner tuples
# 		self.create()


# 	def create(self):
