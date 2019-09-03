import numpy as np
from numpy.linalg import multi_dot
import math
import random


class pointcloud:
	def __init__(self, snap, ground):
		self.snap = snap
		self.ground = ground

	def project_to_surface(*surfaces):
		return 


	def project_to_mesh(*surfaces):
		# define corner/vertex points of the terrain
		# group vertices in pairs nearest neighbors
		# detect if the rays given by self.snap.ray_list intersect
		return 

	# def project_to_ground(self):
	# 	points = []
	# 	for ray in self.snap.rays_to_ground:
	# 		points.append([coord[1] for coord in ray])
	# 	self.camera_perspective(points)
	# 	return points

	# def camera_perspective(self, points):
	# 	transform,_ = self.snap.world_to_camera_transform()
	# 	# transform,_ = self.snap.camera_to_world_transform()
	# 	transform_cloud(points, transform) 

	def project_to_ground(self):
		points = []
		for ray in self.snap.rays_to_ground:
			point = [coord[1] for coord in ray]
			point.append(1)
			points.append(point)
		print(points[0])
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
		print(transform)
		transform_cloud(points, transform) 



####### Ray Tracing ########

def ray_intersect_plane(point1, point2, point3, point4):
		return

# point-normal plane
def ray_intersect_plane2(p0, p1, p_co, p_no, epsilon=1e-6):
    u = p1 - p0
    dot = p_no * u
    if abs(dot) > epsilon:
        w = p0 - p_co
        fac = -(plane * w) / dot
        return p0 + (u * fac)
    else:
        return None

def ray_intersect_plane3():
	epsilon=1e-6

	#Define plane
	planeNormal = np.array([0, 0, 1])
	planePoint = np.array([0, 0, 5]) #Any point on the plane

	#Define ray
	rayDirection = np.array([0, -1, -1])
	rayPoint = np.array([0, 0, 10]) #Any point along the ray

	ndotu = planeNormal.dot(rayDirection) 

	if abs(ndotu) < epsilon:
	    print ("no intersection or line is within plane")

	w = rayPoint - planePoint
	si = -planeNormal.dot(w) / ndotu
	Psi = w + si * rayDirection + planePoint

	print ("intersection at", Psi)


# # intersection function
# def isect_line_plane_v3(p0, p1, p_co, p_no, epsilon=1e-6):
#     """
#     p0, p1: define the line
#     p_co, p_no: define the plane:
#         p_co is a point on the plane (plane coordinate).
#         p_no is a normal vector defining the plane direction;
#              (does not need to be normalized).

#     return a Vector or None (when the intersection can't be found).
#     """

#     u = sub_v3v3(p1, p0)
#     dot = dot_v3v3(p_no, u)

#     if abs(dot) > epsilon:
#         # the factor of the point between p0 -> p1 (0 - 1)
#         # if 'fac' is between (0 - 1) the point intersects with the segment.
#         # otherwise:
#         #  < 0.0: behind p0.
#         #  > 1.0: infront of p1.
#         w = sub_v3v3(p0, p_co)
#         fac = -dot_v3v3(p_no, w) / dot
#         u = mul_v3_fl(u, fac)
#         return add_v3v3(p0, u)
#     else:
#         # The segment is parallel to plane
#         return None

# # ----------------------
# # generic math functions

# def add_v3v3(v0, v1):
#     return (
#         v0[0] + v1[0],
#         v0[1] + v1[1],
#         v0[2] + v1[2],
#         )


# def sub_v3v3(v0, v1):
#     return (
#         v0[0] - v1[0],
#         v0[1] - v1[1],
#         v0[2] - v1[2],
#         )


# def dot_v3v3(v0, v1):
#     return (
#         (v0[0] * v1[0]) +
#         (v0[1] * v1[1]) +
#         (v0[2] * v1[2])
#         )


# def len_squared_v3(v0):
#     return dot_v3v3(v0, v0)


# def mul_v3_fl(v0, f):
#     return (
#         v0[0] * f,
#         v0[1] * f,
#         v0[2] * f,
#         )

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

