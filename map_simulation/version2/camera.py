
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
		self.specs = specs
		self.X = []
		self.Y = []
		self.Z = []
		self.source_x = 0
		self.source_y = 0
		self.source_z = 0
		self.ray_list = []
		self.rays_to_ground = []
		self.create_sensor(**specs)
		self.orient_camera()
		self.project_ray()

	def create_sensor(self, **specs):
		# define pixel grid from specs
		xmax = specs["xmax"]
		ymax = specs["ymax"]
		X_pixel = np.arange(0, xmax, 1)
		Y_pixel = np.arange(0, ymax, 1)
		X, Y = np.meshgrid(X_pixel, Y_pixel)
		transform = self.pixel_to_camera_transform()
		size = X.shape
		axis = (np.zeros(size)+1).flatten()
		sensor = np.dot(transform, [X.flatten(), Y.flatten(), axis])
		self.X = np.reshape(sensor[0],size)
		self.Y = np.reshape(sensor[1],size)

		source_x = specs["focalx"]
		source_y = specs["focaly"]
		source_z = specs["focalz"]
		source = np.dot(transform, [source_x,source_y,1])
		self.source_x = source[0]
		self.source_y = source[1]
		self.source_z = source_z
		
	def orient_camera(self):
		size = self.X.shape
		self.Z = np.zeros(size) 
		R_xyz, offset = self.world_to_camera_transform()
		temp_sensor=np.dot(R_xyz, [self.X.flatten(),self.Y.flatten(),self.Z.flatten()])   
		self.X=np.reshape(temp_sensor[0],size) + offset[0]
		self.Y=np.reshape(temp_sensor[1],size) + offset[1]
		self.Z=np.reshape(temp_sensor[2],size) + offset[2]

		temp_source=np.dot(R_xyz, [self.source_x,self.source_y,self.source_z])
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

	def camera_to_world_transform(self):
		K,offset = self.world_to_camera_transform()
		invoffset = -1*offset
		return np.linalg.inv(K), invoffset

	def camera_to_pixel_transform(self):
		K = np.array([[self.specs["focalx"],0,self.specs["xmax"]/2],[0,self.specs["focaly"],self.specs["ymax"]/2],[0,0,1]])
		return K

	def pixel_to_camera_transform(self):
		K = self.camera_to_pixel_transform()
		return np.linalg.inv(K)

	#####======Projection======#####
	def project_ray(self):
		Cx = self.X.flatten()
		Cy = self.Y.flatten()
		Cz = self.Z.flatten()
		self.ray_list = []
		self.rays_to_ground = []
		for i in range(len(Cx)):
			delta_x = 0
			slope_xz = None
			if self.source_x-Cx[i] != 0:
				slope_xz = (self.source_z-Cz[i])/(self.source_x-Cx[i])
				delta_x = (-self.source_z/slope_xz)

			delta_y = 0
			slope_yz = None
			if self.source_y-Cy[i] != 0:
				slope_yz = (self.source_z-Cz[i])/(self.source_y-Cy[i])
				delta_y = (-self.source_z/slope_yz)

			# self.ray_list.append([slope_xz, slope_yz, 0])
			vector = [delta_x, delta_y, -1*self.source_z]
			self.ray_list.append(vector/np.linalg.norm(vector))
			ray = [[self.source_x, self.source_x+delta_x], [self.source_y, self.source_y+delta_y], [self.source_z, 0]]
			self.rays_to_ground.append(ray)

	# def trace_ray(self):

	# Extract the point cloud from the projected pixels in a separate script for ray projection
	# Feed ray_list along with the surface information into the ray projection script. Handle infinite slope by taking the point directly below
	# Call this ray projection script in visualize.py, after creating the camera, the surfaces, and generating the ray_list 
