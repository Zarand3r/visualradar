You can access the useful functions from the exporter2.py file 

SIMULATION
==========
*Step 1*
To start the simulation, create a new instance of the simulator class.
The constructor requires 6 arguments but accepts a 7th optional argument:
	terrain: stl file path
	xmax: 
	ymax:
	focalx:
	focaly:
	focalz:
	camera_pov: (optional) determines camera coordinates or world coordinates for pointcloud output 
>> sim = simulator("terrains/griffith.stl",30,20,15,10,2)

*Step 2*
To add a camera position, call the add_camera method
The method requires 7 arguments:
	x:
	y:
	z:
	rotx:
	roty:
	rotz:
	time:
>> sim.add_camera(20,10,20,0,0,0,1)
>> sim.add_camera(20,20,20,0,0,0,10)

*Step 3*
To extract point cloud, call the get_pointclouds method
>> pointclouds = sim.get_pointclouds()

*Step 4*
To export to ros bag, call the export_to_bag method
The method requires one parameter, the file path of the bag file to write to
>> sim.export_to_bag("bagfiles/test1.bag")

RAY TRACING
===========
	# # Ray tracing example 
	# points = [[0,0,0],[1,1,1]]
	# vectors = [[0,0,1],[1,1,1]]
	# intersects = trace_many_rays(terrain, points, vectors)
	# print(intersects)

	sim.export_to_bag("bagfiles/test1.bag")



USING IN MATLAB
===============
You can load the functions from exporter2.py in matlab
However, when python is running within MATLAB, it ends up using MATLAB's MKL. Certain libraries such as np.linalg dont work with MATLAB's MKL due to incompatible compile-time options. On linux there is a simple workaround in MATLAB.
>> flag = int32(bitor(2, 8));
>> py.sys.setdlopenflags(flag);



