import matplotlib.pyplot as plt
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
import reconstruction
from itertools import product, combinations


def set_bounds(figure):
	limits = np.array([
		figure.get_xlim(),
		figure.get_ylim(),
		figure.get_zlim(),
	])
	radius = np.max(np.abs(limits))
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
	world = figure.plot_surface(W_x, W_y, W_z, cmap=cm.cool, linewidth=0, antialiased=False)
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


# def plot_projection(rays, figure):
#	visualize point cloud, calculate depth estimate, reconstruct terrain

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

def render():
	# fig = plt.figure(figsize=plt.figaspect(0.5))
	fig = plt.figure(figsize=(16, 8))
	specs = {"xmax":30, "ymax":20, "focalx":15, "focaly":10, "focalz":2} # units of pixel except for focalz ....

	########## First figure axis ##########
	ax1 = fig.add_subplot(1, 2, 1, projection='3d')
	ground = world.ground(100,100)
	plot_world(ax1, ground)
	# cube = world.cube(0,0,10)
	# plot_surfs(ax1, *cube)

	box = world.box((50,-50,20),(50,50,20),(30,50,20),(30,-50,20))
	plot_surfs(ax1, *box)

	orientation2=(0,-20,0)
	snap1 = camera.snap(20, 0, 50, *orientation2, **specs)
	plot_camera(ax1, snap1, True)
	set_bounds(ax1)


	########## Second figure axis ##########
	ax2 = fig.add_subplot(1, 2, 2, projection='3d')
	set_axes_equal(ax1,ax2)

	# pointcloud = projection.pointcloud(snap1)
	# ground_cloud = projection.points_to_pointcloud(pointcloud.project_to_ground())
	# ax2.scatter(*ground_cloud)

	# pointcloud = projection.pointcloud(snap1)
	# box_cloud = projection.points_to_pointcloud(pointcloud.penetrate_plane(*box, camera_pov=False))
	# ax2.scatter(*box_cloud)

	proj2 = projection.pointcloud(snap1)
	pointcloud2 = proj2.project_to_plane(*box, ground, camera_pov=False)
	box_cloud = projection.points_to_pointcloud(pointcloud2)
	ax2.scatter(*box_cloud)


	plt.show()

if __name__== "__main__":
	render()
