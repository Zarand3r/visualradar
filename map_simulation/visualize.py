import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D  
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
from numpy.linalg import multi_dot
import math
import random
import camera
import world
import projection
from itertools import product, combinations
from stl import mesh


def set_bounds(figure, floor=False):
	limits = np.array([
		figure.get_xlim(),
		figure.get_ylim(),
		figure.get_zlim(),
	])

	lim_min = limits[:,0]
	lim_max = limits[:,1]
	radius = np.max(lim_max-lim_min)/2
	origin = np.mean(limits, axis=1)

	if floor:
		ground = world.ground(2*lim_max[0], 2*lim_max[1])
		plot_world(figure, ground)

		limits = np.array([
		figure.get_xlim(),
		figure.get_ylim(),
		figure.get_zlim(),
		])

		lim_min = limits[:,0]
		lim_max = limits[:,1]
		radius = np.max(lim_max-lim_min)/2
		origin = np.mean(limits, axis=1)



	xmin = origin[0] - radius
	xmax = origin[0] + radius
	ymin = origin[1] - radius
	ymax = origin[1] + radius
	zmin = 0
	zmax = 2*radius

	figure.set_xlim([xmin, xmax])
	figure.set_ylim([ymin, ymax])
	figure.set_zlim([zmin, zmax])

	x = [xmin, xmax]
	y = [ymin, ymax]
	z = [zmin, zmax]
	for s, e in combinations(np.array(list(product(x, y, z))), 2):
	    if np.sum(np.abs(s-e)) == x[1]-x[0]:
	        figure.plot3D(*zip(s, e), color="b")


def set_axes_equal(figure1, figure2):
	(xmin,xmax) = figure1.get_xlim()
	(ymin,ymax) = figure1.get_ylim()
	(zmin,zmax) = figure1.get_zlim()
	figure2.set_xlim([xmin, xmax])
	figure2.set_ylim([ymin, ymax])
	figure2.set_zlim([zmin, zmax])

	x = [xmin, xmax]
	y = [ymin, ymax]
	z = [zmin, zmax]
	for s, e in combinations(np.array(list(product(x, y, z))), 2):
	    if np.sum(np.abs(s-e)) == x[1]-x[0]:
	        figure2.plot3D(*zip(s, e), color="b")


def plot_world(figure, world):
	(W_x, W_y, W_z) = (world.X, world.Y, world.Z)
	world = figure.plot_surface(W_x, W_y, W_z, cmap=cm.cool, linewidth=0, antialiased=False, zorder = 0)
	return world

def plot_surfs(figure, *surfaces):
	# figure.set_xlim([surf.xmin, surf.xmax])
	# figure.set_ylim([surf.ymin, surf.ymax])
	for surface in surfaces:
		(W_x, W_y, W_z) = (surface.X, surface.Y, surface.Z)
		figure.plot_surface(W_x, W_y, W_z, linewidth=2, antialiased=False)


def plot_camera(figure, snap, plotrays=True):
	(C_x, C_y, C_z) = (snap.X, snap.Y, snap.Z)
	(source_x, source_y, source_z) = (snap.source_x, snap.source_y, snap.source_z)

	figure.plot_surface(C_x, C_y, C_z, linewidth=0, antialiased=False)
	figure.scatter(source_x, source_y, source_z, color="g", s=20)
	if plotrays:
		lines = snap.rays_to_ground
		for line in lines:
			figure.plot(line[0], line[1], line[2])

def test1():
	# fig = plt.figure(figsize=plt.figaspect(0.5))
	fig = plt.figure(figsize=(16, 8))
	specs = {"xmax":30, "ymax":20, "focalx":15, "focaly":10, "focalz":2} # units of pixel except for focalz ....

	# First figure axis
	ax1 = fig.add_subplot(1, 2, 1, projection='3d')
	ground = world.ground(100,100)
	surf1 = plot_world(ax1, ground)
	
	# plane1 = world.xz_plane(20,20,0,-10,0) 
	# plane2 = world.xz_plane(20,20,0,10, 0)
	# plane3 = world.yz_plane(20,20,-10,0,0)
	# plane4 = world.yz_plane(20,20,10,0,0)
	# plane5 = world.xy_plane(20,20,0,0,20)
	# plot_surfs(ax1, plane1, plane2, plane3, plane4, plane5)
	cube = world.cube(0,0,10)
	plot_surfs(ax1, *cube)
	box = world.box((-50,-50,20),(-50,50,20),(-40,50,20),(-40,-50,20))
	plot_surfs(ax1, *box)


	orientation2=(0,30,0)
	snap1 = camera.snap(0, 0, 30, *orientation2, **specs)
	plot_camera(ax1, snap1, True)
	set_bounds(ax1)
	# fig.colorbar(surf1, shrink=0.5, aspect=5)

def test2():
	# fig = plt.figure(figsize=plt.figaspect(0.5))
	fig = plt.figure(figsize=(16, 8))
	specs = {"xmax":30, "ymax":20, "focalx":15, "focaly":10, "focalz":2} # units of pixel except for focalz ....

	# # First figure axis
	ax1 = fig.add_subplot(1, 2, 1, projection='3d')
	ground = world.ground(100,100)
	plot_world(ax1, ground)
	# cube = world.cube(0,0,10)
	# plot_surfs(ax1, *cube)

	orientation2=(0,30,0)
	snap1 = camera.snap(0, 0, 30, *orientation2, **specs)
	plot_camera(ax1, snap1, True)
	set_bounds(ax1)


	# # Second figure axis 
	ax2 = fig.add_subplot(1, 2, 2, projection='3d')
	set_axes_equal(ax1,ax2)
	pointcloud = projection.pointcloud(snap1, ground)
	ground_cloud = projection.points_to_pointcloud(pointcloud.project_to_ground())
	ax2.scatter(*ground_cloud)


	plt.show()

def test3():

	terrain = "terrains/griffith.stl"
	fig = plt.figure(figsize=(16, 8))
	specs = {"xmax":30, "ymax":20, "focalx":15, "focaly":10, "focalz":2} # units of pixel except for focalz ....

	########## First figure axis ##########
	ax1 = fig.add_subplot(1, 2, 1, projection='3d')
	your_mesh = mesh.Mesh.from_file(terrain)
	ax1.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors))
	scale = your_mesh.points.flatten(-1)
	ax1.auto_scale_xyz(scale, scale, scale)

	orientation1=(0,0,0)
	snap1 = camera.snap(20, 20, 20, *orientation1, **specs)
	orientation2=(0,0,0)
	snap2 = camera.snap(30, 20, 20, *orientation2, **specs)
	plot_camera(ax1, snap1, True)
	plot_camera(ax1, snap2, True)
	set_bounds(ax1)

	########## Second figure axis ##########
	ax2 = fig.add_subplot(1, 2, 2, projection='3d')
	set_axes_equal(ax1,ax2)
	proj1 = projection.pointcloud(snap1)
	pointcloud1 = proj1.project_to_mesh(terrain, convert=True)
	proj2 = projection.pointcloud(snap2)
	pointcloud2 = proj2.project_to_mesh(terrain, convert=True)
	ax2.scatter(*pointcloud1)
	ax2.scatter(*pointcloud2)

	plt.show()



def render(terrain, cameras, pointclouds):
	fig = plt.figure(figsize=(16, 8))

	########## First figure axis ##########
	ax1 = fig.add_subplot(1, 2, 1, projection='3d')
	terrain_mesh = mesh.Mesh.from_file(terrain)
	ax1.add_collection3d(mplot3d.art3d.Poly3DCollection(terrain_mesh.vectors))
	scale = terrain_mesh.points.flatten(-1)
	ax1.auto_scale_xyz(scale, scale, scale)

	for snap in cameras:
		plot_camera(ax1, snap, True)
	set_bounds(ax1)

	########## Second figure axis ##########
	ax2 = fig.add_subplot(1, 2, 2, projection='3d')
	set_axes_equal(ax1,ax2)

	for pointcloud in pointclouds:
		ax2.scatter(*pointcloud)
	
	plt.show()



if __name__== "__main__":
	terrain = "terrains/griffith.stl"
	specs = {"xmax":30, "ymax":20, "focalx":15, "focaly":10, "focalz":2} # units of pixel except for focalz ....

	orientation1=(0,0,0)
	snap1 = camera.snap(20, 20, 20, *orientation1, **specs)
	orientation2=(0,0,0)
	snap2 = camera.snap(30, 20, 20, *orientation2, **specs)

	proj1 = projection.pointcloud(snap1)
	pointcloud1 = proj1.project_to_mesh(terrain, convert=True)
	proj2 = projection.pointcloud(snap2)
	pointcloud2 = proj2.project_to_mesh(terrain, convert=True)

	render(terrain, [snap1, snap2], [pointcloud1, pointcloud2])

