import numpy as np
from numpy.linalg import multi_dot
import math
import random


class surface:
	def __init__(self, boundX, boundY):
		self.xmin = 0
		self.ymin = 0
		self.xmax = boundX
		self.ymax = boundY
		self.X = np.arange(self.xmin, self.xmax, 1)
		self.Y = np.arange(self.ymin, self.ymax, 1)
		self.Z = []
		self.create()

	def create(self):
		self.X, self.Y = np.meshgrid(self.X, self.Y)
		size = self.X.shape
		self.Z = np.zeros(size)

