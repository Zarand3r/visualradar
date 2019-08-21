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
	print(limits)
	radius = 0.5 * np.max(np.abs(limits))
	print(radius)
	origin = np.mean(limits, axis=1)
	figure.set_xlim([origin[0] - radius, origin[0] + radius])
	figure.set_ylim([origin[1] - radius, origin[1] + radius])
	# ax.set_zlim([origin[2] - radius, origin[2] + radius])
	figure.set_zlim([0, 2*radius])
	print(radius)

def plot_world(surf, figure):
	(W_x, W_y, W_z) = (surf.X, surf.Y, surf.Z)
	figure.set_xlim([surf.xmin, surf.xmax])
	figure.set_ylim([surf.ymin, surf.ymax])
	surf = figure.plot_surface(W_x, W_y, W_z, cmap=cm.magma, linewidth=0, antialiased=False)
	return surf


def plot_camera(snap, figure):
	(C_x, C_y, C_z) = (snap.X, snap.Y, snap.Z)
	(S_x, S_y, S_z) = (snap.S_x, snap.S_y, snap.S_z)

	figure.plot_surface(C_x, C_y, C_z, linewidth=0, antialiased=False)
	figure.scatter(S_x, S_y, S_z, color="g", s=20)
	lines = snap.project_ray()
	for line in lines:
		figure.plot(line[0], line[1], line[2])

# def plot_projection(rays, figure):
#	visualize point cloud, calculate depth estimate, reconstruct terrain



def render():
	fig = plt.figure(figsize=plt.figaspect(0.5))

	# First Camera
	# ax1 = fig.gca(projection='3d')
	ax1 = fig.add_subplot(1, 2, 1, projection='3d')
	surf1 = world.surface(300,300)
	snap1 = camera.snap(100, 100, 50, (0,30,0), 20, 30, 30)
	world1 = plot_world(surf1, ax1)
	plot_camera(snap1, ax1)
	set_axes_equal(ax1)
	fig.colorbar(world1, shrink=0.5, aspect=5)

	# Visualize point cloud in different figure axis
	ax2 = fig.add_subplot(1, 2, 2, projection='3d')
	surf2 = world.surface(300,300)
	snap2 = camera.snap(100, 100, 50, (0,30,0), 20, 30, 30)
	world2 = plot_world(surf2, ax2)
	plot_camera(snap2, ax2)
	set_axes_equal(ax2)
	fig.colorbar(world2, shrink=0.5, aspect=5)


	plt.show()

if __name__== "__main__":
	render()
