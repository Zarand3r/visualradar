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
import reconstruction


def set_axes_equal(figure):
	'''Make axes of 3D plot have equal scale so that spheres appear as spheres,
	cubes as cubes, etc..  This is one possible solution to Matplotlib's
	ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

	Input
	  ax: a matplotlib axis, e.g., as output from plt.gca().
	'''

	limits = np.array([
		figure.get_xlim(),
		figure.get_ylim(),
		figure.get_zlim(),
	])
	radius = 0.5 * np.max(np.abs(limits))
	origin = np.mean(limits, axis=1)
	figure.set_xlim([origin[0] - radius, origin[0] + radius])
	figure.set_ylim([origin[1] - radius, origin[1] + radius])
	# ax.set_zlim([origin[2] - radius, origin[2] + radius])
	figure.set_zlim([0, 2*radius])

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
		lines = snap.project_ray()
		for line in lines:
			figure.plot(line[0], line[1], line[2])

# def plot_projection(rays, figure):
#	visualize point cloud, calculate depth estimate, reconstruct terrain



def render():
	fig = plt.figure(figsize=plt.figaspect(0.5))

	specs = {"xmax":30, "ymax":20, "focalx":15, "focaly":10, "focalz":10} # units of pixel except for focalz ....
	# First Camera
	# ax1 = fig.add_subplot(1, 2, 1, projection='3d')
	# orientation1=(0,20,0)
	# surf1 = world.surface(10,10)
	# snap1 = camera.snap(0, 0, 10, *orientation1, **specs)
	# world1 = plot_world(ax1, surf1)
	# plot_camera(ax1, snap1)
	# set_axes_equal(ax1)
	# fig.colorbar(world1, shrink=0.5, aspect=5)

	# Visualize point cloud in different figure axis
	ax2 = fig.add_subplot(1, 2, 2, projection='3d')
	# orientation2=(0,30,0)
	surf2 = world.surface(100,100)
	plane1 = world.xz_plane(20,20,0,-10,0) 
	plane2 = world.xz_plane(20,20,0,10, 0)
	plane3 = world.yz_plane(20,20,-10,0,0)
	plane4 = world.yz_plane(20,20,10,0,0)
	plane5 = world.xy_plane(20,20,0,0,20)
	# snap2 = camera.snap(100, 100, 50, *orientation2, **specs)
	world2 = plot_world(ax2, surf2)
	plot_surfs(ax2, plane1, plane2, plane3, plane4, plane5)
	# plot_camera(snap2, ax2, False)
	set_axes_equal(ax2)
	fig.colorbar(world2, shrink=0.5, aspect=5)


	plt.show()

if __name__== "__main__":
	render()
