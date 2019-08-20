import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
from numpy.linalg import multi_dot
import math
import random
import calibration as cal

# make camera object with the parameters as static instance variables
# make surface object 
# define a list of cameras each at a different position
# define a list of surfaces allows us to make more complex surfaces 

def set_axes_equal(ax):
	'''Make axes of 3D plot have equal scale so that spheres appear as spheres,
	cubes as cubes, etc..  This is one possible solution to Matplotlib's
	ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

	Input
	  ax: a matplotlib axis, e.g., as output from plt.gca().
	'''

	limits = np.array([
		[cal.world_xmin, cal.world_xmax],
		[cal.world_ymin, cal.world_ymax],
		ax.get_zlim(),
	])
	print(limits)
	radius = 0.5 * np.max(np.abs(limits))
	print(radius)
	origin = np.mean(limits, axis=1)
	ax.set_xlim([origin[0] - radius, origin[0] + radius])
	ax.set_ylim([origin[1] - radius, origin[1] + radius])
	# ax.set_zlim([origin[2] - radius, origin[2] + radius])
	ax.set_zlim([0, 2*radius])
	print(radius)

def get_rotation_matrix(theta,phi,gamma):
	theta = math.radians(theta)
	phi = math.radians(phi)
	gamma = math.radians(gamma)

	c1=math.cos(theta)
	c2=math.cos(phi)
	c3=math.cos(gamma)
	s1=math.sin(theta)
	s2=math.sin(phi)
	s3=math.sin(gamma)
	# R_zyx=[[c1*c2, c1*s2*s3-c3*s1, s1*s3+c1*c3*s2], [c2*s1,c1*c3+s1*s2*s3,c3*s1*s2-c1*s3], [-s2,c2*s3,c2*c3]]
	R_x=[[1,0,0],[0,c1,-s1],[0,s1,c1]]
	R_y=[[c2,0,s2],[0,1,0],[-s2,0,c2]]
	R_z=[[c3,-s3,0],[s3,c3,0],[0,0,1]]
	R_xyz = multi_dot([R_x, R_y, R_z])
	return R_xyz

def rotate_mesh(X,Y,Z,X_pos,Y_pos,Z_pos,theta,phi,gamma):
	size = X.shape
	R_xyz = get_rotation_matrix(theta, phi, gamma)
	temp=np.dot(np.transpose(R_xyz), [X.flatten(),Y.flatten(),Z.flatten()])    
	Xrot=np.reshape(temp[0],size)
	Yrot=np.reshape(temp[1],size)
	Zrot=np.reshape(temp[2],size)
	X = Xrot + X_pos
	Y = Yrot + Y_pos 
	Z = Zrot + Z_pos
	return (X, Y, Z)

def camera(X_pos=cal.camera_x, Y_pos=cal.camera_y, Z_pos=cal.camera_z, rot=cal.camera_rot, width=cal.camera_w, length=cal.camera_l):
	X = np.arange(0, width, 1)
	Y = np.arange(0, length, 1)
	X, Y = np.meshgrid(X, Y)
	size = X.shape
	Z = np.zeros(size)
	X,Y,Z = rotate_mesh(X,Y,Z,X_pos,Y_pos,Z_pos,rot[0],rot[1], rot[2])
	return (X,Y,Z)

def sensor(X_pos=cal.camera_x, Y_pos=cal.camera_y, Z_pos=cal.camera_z, rot=cal.camera_rot, width=cal.camera_w, length=cal.camera_l, focal=cal.camera_f):
	R_xyz = get_rotation_matrix(rot[0],rot[1],rot[2])
	temp=np.dot(np.transpose(R_xyz), [width/2,length/2,focal])
	X = temp[0] + X_pos
	Y = temp[1] + Y_pos 
	Z = temp[2] + Z_pos
	return (X,Y,Z)


def project_ray(C_x, C_y, C_z, S_x, S_y, S_z):
	Cx = C_x.flatten()
	Cy = C_y.flatten()
	Cz = C_z.flatten()
	ray_list = []
	for i in range(len(Cx)):
		slope_xz = (S_z-Cz[i])/(S_x-Cx[i])
		slope_yz = (S_z-Cz[i])/(S_y-Cy[i])
		delta_x = (-S_z/slope_xz)
		delta_y = (-S_z/slope_yz)
		ray = [[S_x, S_x+delta_x], [S_y, S_y+delta_y], [S_z, 0]]
		ray_list.append(ray)
	return ray_list

# draws the surface, with Z defined as a function of X and Y
def surface(min_x=cal.world_xmin, max_x=cal.world_xmax, min_y=cal.world_ymin, max_y=cal.world_ymax,):
	X = np.arange(min_x, max_x, 1)
	Y = np.arange(min_y, max_y, 1)
	X, Y = np.meshgrid(X, Y)
	size = X.shape
	Z = np.zeros(size)
	return (X, Y, Z)

def render():
	fig = plt.figure()
	ax = fig.gca(projection='3d')
	ax.zaxis.set_major_locator(LinearLocator(10))
	ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
	# Plot the surface.
	C_x,C_y,C_z = camera()
	S_x,S_y,S_z = sensor()
	W_x,W_y,W_z = surface()
	ax.plot_surface(C_x, C_y, C_z, linewidth=0, antialiased=False)
	ax.scatter(S_x, S_y, S_z, color="g", s=20)
	surf = ax.plot_surface(W_x, W_y, W_z, cmap=cm.magma, linewidth=0, antialiased=False)

	lines = project_ray(C_x, C_y, C_z, S_x, S_y, S_z)
	for line in lines:
		ax.plot(line[0], line[1], line[2])
	set_axes_equal(ax)
	fig.colorbar(surf, shrink=0.5, aspect=5)
	plt.show()

def visualize():
	fig = plt.figure()
	ax = fig.gca(projection='3d')
	#plot the projected points what the camera sees
	plt.show()


if __name__== "__main__":
	render()