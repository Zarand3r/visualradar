import numpy as np
from numpy.linalg import multi_dot
import math
import random
import camera
import world
import projection
import visualize
import exporter
from itertools import product, combinations
import stl
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
set_trajectory generates camera objects in between the latest camera position and a specified final position
the number camera objects generated in between is determined by the frames parameter
'''
class simulator:
	def __init__(self, terrain, **specs):
		self.terrain = terrain
		self.specs = specs
		self.cameras = []
		self.projections = []
		self.pointclouds = []
		self.filename = ''
		# if filename:
		# 	self.export = True

	def export_to_bag(self, filename):
		self.filename = filename

	def add_camera(self, x, y, z, rotx, roty, rotz):
		orientation = (rotx,roty,rotz)
		snap = camera.snap(x, y, z, *orientation, **self.specs)
		self.cameras.append(snap)
		proj = projection.pointcloud(snap)
		self.projections.append(proj)
		if self.filename:
			pointcloud = proj.project_to_mesh(self.terrain, camera_pov=False, convert=False)
			pointsmessage = exporter.xyz_array_to_pointcloud2(pointcloud)
			exporter.make_bag_file(self.filename, pointsmessage)

	def set_trajectory(self, final_x, final_y, final_z, final_rotx, final_roty, final_rotz, frames=3):
		current = self.cameras[-1]
		dx = (final_x-current.X_pos)/frames
		dy = (final_y-current.Y_pos)/frames
		dz = (final_z-current.Z_pos)/frames
		drx = (final_rotx-current.theta)/frames
		dry = (final_roty-current.phi)/frames
		drz = (final_rotz-current.gamma)/frames

		for i in range(frames):
			self.add_camera(current.X_pos+dx, current.Y_pos+dy, current.Z_pos+dz, current.theta+drx, current.phi+dry, current.gamma+drz)
			current = self.cameras[-1]


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


def find_mins_maxs(terrain):
	terrain_mesh = mesh.Mesh.from_file(terrain)
	minx = maxx = miny = maxy = minz = maxz = None
	for p in terrain_mesh.points:
		# p contains (x, y, z)
		if minx is None:
			minx = p[stl.Dimension.X]
			maxx = p[stl.Dimension.X]
			miny = p[stl.Dimension.Y]
			maxy = p[stl.Dimension.Y]
			minz = p[stl.Dimension.Z]
			maxz = p[stl.Dimension.Z]
		else:
			maxx = max(p[stl.Dimension.X], maxx)
			minx = min(p[stl.Dimension.X], minx)
			maxy = max(p[stl.Dimension.Y], maxy)
			miny = min(p[stl.Dimension.Y], miny)
			maxz = max(p[stl.Dimension.Z], maxz)
			minz = min(p[stl.Dimension.Z], minz)
	return minx, maxx, miny, maxy, minz, maxz


if __name__== "__main__":
	terrain = "terrains/griffith.stl"

	# # Ray tracing example 
	# points = [[0,0,0],[1,1,1]]
	# vectors = [[0,0,1],[1,1,1]]
	# intersects = trace_many_rays(terrain, points, vectors)
	# print(intersects)

	# Simulation example
	specs = {"xmax":30, "ymax":20, "focalx":15, "focaly":10, "focalz":2} # units of pixel except for focalz ....
	sim = simulator(terrain, **specs)
	sim.export_to_bag("bagfiles/test1.bag")
	sim.add_camera(20,10,20,0,0,0)
	sim.add_camera(20,20,20,0,0,0)
	# sim.set_trajectory(30,40,10,0,-10,0)
	cameras = sim.get_cameras()
	pointclouds = sim.get_pointclouds()
	# sim.plot()


