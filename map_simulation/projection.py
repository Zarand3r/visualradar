import numpy as np
from numpy.linalg import multi_dot
import math
import random

import os
import vtk
from pycaster import pycaster

class pointcloud:
	def __init__(self, snap):
		self.snap = snap

	def project_to_plane(self, surfaces, camera_pov=False, convert=False):
		points = []
		source = [self.snap.source_x, self.snap.source_y, self.snap.source_z]
		for ray in self.snap.ray_list:
			intersects = []
			for surface in surfaces:	
				vertices = [surface.corner1, surface.corner2, surface.corner3, surface.corner4]
				ray_vector = [ray[0], ray[1], ray[2]]
				point = ray_intersect_plane(source, vertices, ray_vector)
				bounds = plane_bounds(vertices)
				if point is not None and point_on_plane(point, *bounds):
					intersects.append(point)
			if not intersects:
				continue
			closest = closest_point(source, *intersects)
			points.append(closest)
		points = np.insert(points, 3, 1, axis=1)
		if camera_pov:
			self.camera_perspective(points)
		if convert:
			return points_to_pointcloud(points)
		return points

	def penetrate_plane(self, surfaces, camera_pov=True): ########
		points = []
		source = [self.snap.source_x, self.snap.source_y, self.snap.source_z]
		for ray in self.snap.ray_list:
			for surface in surfaces:	
				vertices = [surface.corner1, surface.corner2, surface.corner3, surface.corner4]
				ray_vector = [ray[0], ray[1], ray[2]]
				point = ray_intersect_plane(source, vertices, ray_vector)
				bounds = plane_bounds(vertices)
				if point is not None and point_on_plane(point, *bounds):
					points.append(point)				
		points = np.insert(points, 3, 1, axis=1)
		if camera_pov:
			self.camera_perspective(points)
		return points


	def project_to_mesh(self, terrain, camera_pov=False, convert=False):
		points = []
		source = [self.snap.source_x, self.snap.source_y, self.snap.source_z]
		for ray in self.snap.rays_to_ground:
			ray = np.array(ray)
			caster = pycaster.rayCaster.fromSTL(terrain, scale=1)
			intersects = caster.castRay(source, ray[:,1])
			if not intersects:
				continue
			closest = closest_point(source, *intersects)
			points.append(closest)
		points = np.insert(points, 3, 1, axis=1)
		if camera_pov:
			self.camera_perspective(points)
		if convert:
			return points_to_pointcloud(points)
		return points

	def project_to_ground(self, camera_pov=False):
		points = []
		for ray in self.snap.rays_to_ground:
			point = [coord[1] for coord in ray]
			point.append(1)
			points.append(point)
		if camera_pov:
			self.camera_perspective(points)
		return points

	def camera_perspective(self, points):
		# transform,_ = self.snap.world_to_camera_transform()
		rotation = self.snap.get_rotation_matrix().tolist()
		rotation[0].append(-1*self.snap.X_pos)
		rotation[1].append(-1*self.snap.Y_pos)
		rotation[2].append(self.snap.Z_pos)
		rotation.append([0,0,0,1])
		transform = np.array(rotation)
		transform_cloud(points, transform) 



####### Ray Tracing ########

def ray_intersect_plane(source, vertices, ray_vector): ########
	epsilon=1e-6
	p1 = np.array(vertices[0])
	p2 = np.array(vertices[1])
	p3 = np.array(vertices[2])
	p4 = np.array(vertices[3])
	# These two vectors are in the plane
	v1 = p3 - p1
	v2 = p2 - p1
	# the cross product is a vector normal to the plane
	cp = np.cross(v1, v2)
	#Define plane
	planeNormal = np.array(cp)
	planePoint = np.array(p4) #Any point on the plane

	#Define ray
	rayDirection = np.array([ray_vector[0], ray_vector[1], ray_vector[2]])
	rayPoint = np.array(source) #Any point along the ray

	# if None in rayDirection:
	# 	return ## deal with this later ... happens when either slopexz or slope yz is infinite 
	# else:
		# ndotu = planeNormal.dot(rayDirection) 

	ndotu = planeNormal.dot(rayDirection) 

	if abs(ndotu) < epsilon:
	    return

	w = rayPoint - planePoint
	si = -planeNormal.dot(w) / ndotu
	Psi = w + si * rayDirection + planePoint

	return Psi

def ray_intersect_mesh(terrain, point1, point2):
	caster = pycaster.rayCaster.fromSTL(terrain, scale=1)
	intersects = caster.castRay(point1, point2)
	return intersects

def plane_bounds(vertices):
	minx = min(vertex[0] for vertex in vertices)
	maxx = max(vertex[0] for vertex in vertices)
	miny = min(vertex[1] for vertex in vertices)
	maxy = max(vertex[1] for vertex in vertices)
	minz = min(vertex[2] for vertex in vertices)
	maxz = max(vertex[2] for vertex in vertices)
	return (minx, maxx, miny, maxy, minz, maxz)

def point_on_plane(point, minx, maxx, miny, maxy, minz, maxz): ########
	#check if point is in the range given by the vertices 
	if minx<=point[0]<=maxx and miny<=point[1]<=maxy and minz<=point[2]<=maxz:
		return True
	return False

def squared_distance_point(point1, point2):
	x1,y1,z1 = point1
	x2,y2,z2 = point2
	p1 = np.array([x1, y1, z1])
	p2 = np.array([x2, y2, z2])
	squared_dist = np.sum((p1-p2)**2, axis=0)
	return squared_dist

def closest_point(point1, *points):
	closest = points[0]
	dist = squared_distance_point(point1, closest)
	if len(points) > 1:
		for point in points:
			if squared_distance_point(point1, point) < dist:
				closest = point 
				dist = squared_distance_point(point1, closest)
	return closest

def furthest_point(point1, *points):
	furthest = points[0]
	dist = squared_distance_point(point1, furthest)
	if len(points) > 1:
		for point in points:
			if squared_distance_point(point1, point) > dist:
				furthest = point 
				dist = squared_distance_point(point1, furthest)
	return furthest


####### HELPER FUNCTIONS #######

def points_to_pointcloud(points):
	X = []
	Y = []
	Z = []
	# X.append(point[0] for point in points)
	# Y.append(point[1] for point in points)
	# Z.append(point[2] for point in points)
	for point in points:
		X.append(point[0])
		Y.append(point[1])
		Z.append(point[2])
	return (X,Y,Z)

def transform_cloud(points, transform):
	for index, point in enumerate(points):
		points[index] = np.dot(transform, point)




if __name__== "__main__":
	ray_intersect_plane3()

