
import numpy as np
from numpy.linalg import multi_dot
import math
import random

# make camera object with the parameters as static instance variables
# make surface object 
# define a list of cameras each at a different position
# define a list of surfaces allows us to make more complex surfaces 

# Rename what used to e called the sensor to the source
# Based on specs dictionary, make a pixel array
# Use transformation to make the corresponding sensor grid 
# Feed the matrkx grid into the orienting functions to convert to camera coordinates 

class snap:
	def __init__(self, camera_x, camera_y, camera_z, *orientation, **specs):
		self.X_pos = camera_x
		self.Y_pos = camera_y
		self.Z_pos = camera_z
		self.theta = math.radians(orientation[0])
		self.phi = math.radians(orientation[1])
		self.gamma = math.radians(orientation[2])
		self.sensor = []
		self.focal = specs["focal"]
		self.X = []
		self.Y = []
		self.Z = []
		self.source_x = 0
		self.source_y = 0
		self.source_z = self.focal
		self.create_sensor(**specs)
		self.orient_camera()

	def create_sensor(self, **specs):
		# define pixel grid from specs
		xmax = specs["xmax"]
		ymax = specs["ymax"]
		X_pixel = np.arange(0, xmax, 1)
		Y_pixel = np.arange(0, xmax, 1)
		# use pixel_to_camera_transform
		# create sensor by transforming pixel grid to camera
		# transform source coordinates too (0,0,focal) gets transformed to origin of the camera coordinate
		self.X = X_pixel
		self.Y = X_pixel

	def orient_camera(self):
		X, Y = np.meshgrid(self.X, self.Y)
		size = X.shape
		Z = np.zeros(size) 
		R_xyz, offset = self.world_to_camera_transform()
		temp_sensor=np.dot(np.transpose(R_xyz), [X.flatten(),Y.flatten(),Z.flatten()])   
		self.X=np.reshape(temp_sensor[0],size) + offset[0]
		self.Y=np.reshape(temp_sensor[1],size) + offset[1]
		self.Z=np.reshape(temp_sensor[2],size) + offset[2]
		temp_source=np.dot(np.transpose(R_xyz), [self.source_x,self.source_y,self.source_z])
		self.source_x = temp_source[0] + offset[0]
		self.source_y = temp_source[1] + offset[1]
		self.source_z = temp_source[2] + offset[2]

	#####======TRANSFORMS======#####

	def get_rotation_matrix(self):
		c1=math.cos(self.theta)
		c2=math.cos(self.phi)
		c3=math.cos(self.gamma)
		s1=math.sin(self.theta)
		s2=math.sin(self.phi)
		s3=math.sin(self.gamma)
		# R_zyx=[[c1*c2, c1*s2*s3-c3*s1, s1*s3+c1*c3*s2], [c2*s1,c1*c3+s1*s2*s3,c3*s1*s2-c1*s3], [-s2,c2*s3,c2*c3]]
		R_x=[[1,0,0],[0,c1,-s1],[0,s1,c1]]
		R_y=[[c2,0,s2],[0,1,0],[-s2,0,c2]]
		R_z=[[c3,-s3,0],[s3,c3,0],[0,0,1]]
		R_xyz = multi_dot([R_x, R_y, R_z])
		return R_xyz

	def world_to_camera_transform(self):
		R_xyz = self.get_rotation_matrix()
		offset = [self.X_pos, self.Y_pos, self.Z_pos]
		return (R_xyz, offset)
	def pixel_to_camera_transform(self):
		matrix = [[],[],[]]
		return 

	#####======Projection======#####
	def project_ray(self):
		Cx = self.X.flatten()
		Cy = self.Y.flatten()
		Cz = self.Z.flatten()
		ray_list = []
		for i in range(len(Cx)):
			slope_xz = (self.source_z-Cz[i])/(self.source_x-Cx[i])
			slope_yz = (self.source_z-Cz[i])/(self.source_y-Cy[i])
			delta_x = (-self.source_z/slope_xz)
			delta_y = (-self.source_z/slope_yz)
			ray = [[self.source_x, self.source_x+delta_x], [self.source_y, self.source_y+delta_y], [self.source_z, 0]]
			ray_list.append(ray)
		return ray_list
	# Extract the point cloud from the projected pixels in a separate script for ray projection
	# Feed ray_list along with the surface information into the ray projection script. Handle infinite slope by taking the point directly below
	# Call this ray projection script in visualize.py, after creating the camera, the surfaces, and generating the ray_list 
