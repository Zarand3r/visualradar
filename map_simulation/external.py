import numpy as np
from numpy.linalg import multi_dot
import math
import random
import camera
import world
import projection
import visualize
from itertools import product, combinations
from stl import mesh


'''
Input: terrain file path and specs = {"xmax","ymax","focalx","focaly","focalz"}
Add camera objects to the simulation and specify position,orientation with add_camera
Output: list of pointclouds corresponding to the camera objects 
Note: Optional parameters in get_pointclouds are transform and convert.
camera_pov = True means pointcloud is returned in camera coordinates
camera_pov = False means pointcloud is returned in world coordinates
convert = True means point cloud is returned as three separate x, y, and z coordinate lists
convert = False means point cloud is returned as a single list of (x,y,z) coordinate tuples
'''
class simulator:
	def __init__(self, terrain, **specs):
		self.terrain = terrain
		self.specs = specs
		self.cameras = []
		self.projections = []
		self.pointclouds = []

	def add_camera(self, x, y, z, rotx, roty, rotz):
		orientation = (rotx,roty,rotz)
		snap = camera.snap(x, y, z, *orientation, **self.specs)
		self.cameras.append(snap)
		proj = projection.pointcloud(snap)
		self.projections.append(proj)

	def get_cameras(self):
		return self.cameras

	def get_pointclouds(self, camera_pov = False, convert=True):
		for proj in self.projections:
			pointcloud = proj.project_to_mesh(self.terrain, camera_pov=camera_pov, convert=convert)
			self.pointclouds.append(pointcloud)
		return self.pointclouds

	def plot(self):
		visualize.render(self.terrain, self.cameras, self.pointclouds)


'''
Input: terrain (stl file path), source (point tuple), vector (vector tuple)
Output: point tuple corresponding to where the vector projects the source point onto the mesh
Note: function converts vector to unit length 
'''
def trace_ray(terrain, source, vector, plot=False):
	vector = vector/np.linalg.norm(vector)
	terrain_mesh = mesh.Mesh.from_file(terrain)
	terrain_points = terrain_mesh.points[:,2]
	height = max(terrain_points)

	point = None

	if vector[2] > 0:
		if source[2] < height:
			point1 = source
			scaler = 2*(height-source[2])/vector[2]
			x2 = source[0]+scaler*vector[0]
			y2 = source[1]+scaler*vector[1]
			z2 = source[2]+scaler*vector[2]
			point2 = [x2, y2, z2]
			intersects = projection.ray_intersect_mesh(terrain,point1,point2)
			point = projection.furthest_point(point1, *intersects)

	if plot:
		import matplotlib.pyplot as plt
		from mpl_toolkits import mplot3d
		from mpl_toolkits.mplot3d import Axes3D 
		fig = plt.figure(figsize=(8, 8)) 
		ax1 = fig.add_subplot(1, 1, 1, projection='3d')
		ax1.add_collection3d(mplot3d.art3d.Poly3DCollection(terrain_mesh.vectors))
		scale = terrain_mesh.points.flatten(-1)
		ax1.auto_scale_xyz(scale, scale, scale)
		ax1.plot([point1[0], point2[0]], [point1[1], point2[1]], [point1[2], point2[2]])
		plt.show()

	return point

'''
Input: terrain (stl file path), sources (list of point tuples), vectors (list of vector tuples)
Output: list of point tuples, corresponding to where each respective point and vector project onto the mesh
Note: function converts vector to unit length 
'''
def trace_many_rays(terrain, sources, vectors):
	points = []
	for index, source in enumerate(sources):
		point = trace_ray(terrain,source,vectors[index])
		points.append(point)
	return points

if __name__== "__main__":
	terrain = "terrains/griffith.stl"

	# Ray tracing example 
	points = [[0,0,0],[1,1,1]]
	vectors = [[0,0,1],[1,1,1]]
	intersects = trace_many_rays(terrain, points, vectors)
	print(intersects)

	# Simulation example
	specs = {"xmax":30, "ymax":20, "focalx":15, "focaly":10, "focalz":2} # units of pixel except for focalz ....
	sim = simulator(terrain, **specs)
	sim.add_camera(20,20,20,0,0,0)
	sim.add_camera(30,20,20,0,0,0)
	cameras = sim.get_cameras()
	pointclouds = sim.get_pointclouds()
	sim.plot()



