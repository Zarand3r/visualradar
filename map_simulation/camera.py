
import numpy as np
from numpy.linalg import multi_dot
import math
import random

# make camera object with the parameters as static instance variables
# make surface object 
# define a list of cameras each at a different position
# define a list of surfaces allows us to make more complex surfaces 

class snap:
	def __init__(self, camera_x, camera_y, camera_z, rot, width, length, focal):
		self.X_pos = camera_x
		self.Y_pos = camera_y
		self.Z_pos = camera_z
		self.w = width
		self.l = length
		self.f = focal
		self.theta = math.radians(rot[0])
		self.phi = math.radians(rot[1])
		self.gamma = math.radians(rot[2])
		self.X = np.arange(0, width, 1)
		self.Y = np.arange(0, length, 1)
		self.Z = []
		self.orient_camera()


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

	def orient_camera(self):
		X, Y = np.meshgrid(self.X, self.Y)
		size = X.shape
		Z = np.zeros(size) 
		R_xyz, offset = self.world_to_camera_transform()
		temp=np.dot(np.transpose(R_xyz), [X.flatten(),Y.flatten(),Z.flatten()])   
		self.X=np.reshape(temp[0],size) + offset[0]
		self.Y=np.reshape(temp[1],size) + offset[1]
		self.Z=np.reshape(temp[2],size) + offset[2]


