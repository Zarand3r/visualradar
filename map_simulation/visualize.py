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
	# (S_x, S_y, S_z) = snap.sensor()

	figure.plot_surface(C_x, C_y, C_z, linewidth=0, antialiased=False)
	# ax.scatter(S_x, S_y, S_z, color="g", s=20)

# def plot_projection(rays, figure):
# 	S_x,S_y,S_z = sensor()
# 	W_x,W_y,W_z = surface()
	
# 	surf = ax.plot_surface(W_x, W_y, W_z, cmap=cm.magma, linewidth=0, antialiased=False)

# 	lines = project_ray(C_x, C_y, C_z, S_x, S_y, S_z)
# 	for line in lines:
# 		ax.plot(line[0], line[1], line[2])

def render():
	fig = plt.figure()
	ax = fig.gca(projection='3d')
	ax.zaxis.set_major_locator(LinearLocator(10))
	ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
	
	surf1 = world.surface(300,300)
	snap1 = camera.snap(100, 100, 50, (0,30,0), 20, 30, 30)
	# rays = reconstruction.project()
	world1 = plot_world(surf1, ax)
	plot_camera(snap1, ax)
	# plot_projection(rays1, ax)
	set_axes_equal(ax)
	fig.colorbar(world1, shrink=0.5, aspect=5)

	# visualize point cloud in different figure axis
	plt.show()

if __name__== "__main__":
	render()