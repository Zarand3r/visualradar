import external
import numpy
import rosbag


def make_bag_file(filename, pointcloud, pose, timestamp=None):
	try:
		bag = rosbag.Bag(filename, 'a')
	except:
		bag = rosbag.Bag(filename, 'w')

	try:
	    # i = Int32()
	    # i.data = 42

	    bag.write('pointcloud', pointcloud, timestamp)
	    bag.write('pose', pose, timestamp)
	finally:
	    bag.close()
	return

# Author: Jon Binney
'''
Functions for working with PointCloud2.
'''

import roslib; # roslib.load_manifest('pr2_python')
import numpy as np, struct
from sensor_msgs.msg import PointCloud2, PointField
from tf import transformations
from geometry_msgs.msg import Transform, Vector3, Quaternion
import numpy.lib.recfunctions as nlr
from itertools import izip

def pointcloud2_to_array(cloud_msg):
    ''' 
    Converts a rospy PointCloud2 message to a numpy recordarray 
    
    Assumes all fields 32 bit floats, and there is no padding.
    '''
    dtype_list = [(f.name, np.float32) for f in cloud_msg.fields]
    cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
    return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

def get_xyz_points(cloud_array, remove_nans=True):
    '''
    Pulls out x, y, and z columns from the cloud recordarray, and returns a 3xN matrix.
    '''
    # remove crap points
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]
    
    # pull out x, y, and z values
    points = np.zeros(list(cloud_array.shape) + [3], dtype=np.float)
    points[...,0] = cloud_array['x']
    points[...,1] = cloud_array['y']
    points[...,2] = cloud_array['z']

    return points

def pointcloud2_to_xyz_array(cloud_msg, remove_nans=True):
    return get_xyz_points(pointcloud2_to_array(cloud_msg), remove_nans=remove_nans)

def xyz_array_to_pointcloud2(points, stamp=None, frame_id=None):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()
    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12*points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = np.asarray(points, np.float32).tostring()

    return msg


def xyzrgb_array_to_pointcloud2(points, colors, stamp=None, frame_id=None, seq=None):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()
    assert(points.shape == colors.shape)

    buf = []

    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if seq: 
        msg.header.seq = seq
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        N = len(points)
        xyzrgb = np.array(np.hstack([points, colors]), dtype=np.float32)
        msg.height = 1
        msg.width = N

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('r', 12, PointField.FLOAT32, 1),
        PointField('g', 16, PointField.FLOAT32, 1),
        PointField('b', 20, PointField.FLOAT32, 1)
    ]
    msg.is_bigendian = False
    msg.point_step = 24
    msg.row_step = msg.point_step * N
    msg.is_dense = True; 
    msg.data = xyzrgb.tostring()

    return msg


if __name__== "__main__":
	terrain = "terrains/griffith.stl"
	# Simulation example
	specs = {"xmax":30, "ymax":20, "focalx":15, "focaly":10, "focalz":2} # units of pixel except for focalz ....
	sim = external.simulator(terrain, **specs)
	sim.add_camera(20,10,20,0,0,0)
	# sim.add_camera(20,20,20,0,0,0)
	# sim.set_trajectory(30,40,10,0,-10,0)
	cameras = sim.get_cameras()
	points = sim.get_pointclouds(convert=False)
	points = np.array(points)
	pointsmessage = xyz_array_to_pointcloud2(points)
	make_bag_file("test", pointsmessage)
