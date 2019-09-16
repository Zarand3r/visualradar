import numpy as np
from numpy.linalg import multi_dot
import math
import random
import camera
import world
import projection
import reconstruction
import visualize
from itertools import product, combinations
from stl import mesh

## export pointcloud array for matthias 

class simulator:
	def __init__(self, terrain, **specs):
		self.terrain = terrain
		self.specs = specs
		self.cameras = []
		self.pointclouds = []

	def add_camera(self, x, y, z, rotx, roty, rotz):
		orientation = (rotx,roty,rotz)
		snap = camera.snap(x, y, z, *orientation, **self.specs)
		self.cameras.append(snap)
		proj = projection.pointcloud(snap)
		pointcloud = proj.project_to_mesh(self.terrain)
		self.pointclouds.append(pointcloud)

	def get_cameras(self):
		return self.cameras

	def get_pointclouds(self, convert=True):
		if convert:
			for index, pointcloud in enumerate(self.pointclouds):
				self.pointclouds[index] = projection.points_to_pointcloud(pointcloud)
		return self.pointclouds

	def plot(self):
		visualize.render(self.terrain, self.cameras, self.pointclouds)

# def trace_ray(point1, point2, terrain):



if __name__== "__main__":
	terrain = "terrains/griffith.stl"
	specs = {"xmax":30, "ymax":20, "focalx":15, "focaly":10, "focalz":2} # units of pixel except for focalz ....

	sim = simulator(terrain, **specs)
	sim.add_camera(20,20,20,0,0,0)
	sim.add_camera(30,20,20,0,0,0)
	cameras = sim.get_cameras()
	pointclouds = sim.get_pointclouds()

	sim.plot()

