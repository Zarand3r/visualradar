import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
import random

def camera(X_pos, Y_pos, Z_pos, rot, width=20, length=30):
	X = np.arange(X_pos-1*width/2, X_pos+width/2, 1)
	Y = np.arange(Y_pos-1*length/2, Y_pos+length/2, 1)
	X, Y = np.meshgrid(X, Y)

	Z = np.zeros(X.shape)
	Z = Z + Z_pos
	Z = Z + np.sin(rot)*X  # clockwise
	return (X, Y, Z)

#makes the surface, with Z defined as a function of X and Y
def surface(bound_X=100, bound_Y=100):
	X = np.arange(-1*bound_X, bound_X, 0.25)
	Y = np.arange(-1*bound_Y, bound_Y, 0.25)
	X, Y = np.meshgrid(X, Y)
	Z = 0.1*(X+Y)
	return (X, Y, Z)

def render():
	fig = plt.figure()
	ax = fig.gca(projection='3d')
	ax.zaxis.set_major_locator(LinearLocator(10))
	ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
	# Plot the surface.
	C_x,C_y,C_z = camera(20, 20, 100, 0)
	S_x,S_y,S_z = surface()
	cam = ax.plot_surface(C_x, C_y, C_z, linewidth=0, antialiased=False)
	surf = ax.plot_surface(S_x, S_y, S_z, cmap=cm.magma, linewidth=0, antialiased=False)
	fig.colorbar(surf, shrink=0.5, aspect=5)
	plt.show()


if __name__== "__main__":
	render()