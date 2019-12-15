# This script interacts with the vehicle carla object and gets its information which
# is used to form various transformation matrices. It also gives bounding boxes of
# vehicle objects.

import numpy as np


def make_transformation(x, y, z, roll, pitch, yaw):
	''' Form homogeneous transformation'''
	# Convert to ROS right handed convention
	R_x = np.array(
		[
			[1, 0, 0],
			[0, np.cos(roll), -np.sin(roll)],
			[0, np.sin(roll), np.cos(roll)],
		]
	)

	R_y = np.array(
		[
			[np.cos(pitch), 0, np.sin(pitch)],
			[0, 1, 0],
			[-np.sin(pitch), 0, np.cos(pitch)],
		]
	)

	R_z = np.array(
		[
			[np.cos(yaw), -np.sin(yaw), 0],
			[np.sin(yaw), np.cos(yaw), 0],
			[0, 0, 1],
		]
	)

	R = np.matmul(R_z, np.matmul(R_y, R_x))
	t = np.array([x, y, z])

	H = np.c_[R, t]
	base = np.array([0, 0, 0, 1])
	H = np.r_[H, [base]]

	return H


def extract_actor_info(vehicle):
	''' This function extracts info such as position and orientation'''
	# Getting position and converting to ROS Frame
	x = vehicle.get_transform().location.x
	y = -vehicle.get_transform().location.y
	z = vehicle.get_transform().location.z
	
	# Getting orientation and converting to radians
	roll = -(vehicle.get_transform().rotation.roll) * np.pi / 180
	pitch = -(vehicle.get_transform().rotation.pitch) * np.pi / 180
	yaw = -(vehicle.get_transform().rotation.yaw) * np.pi / 180

	return [x, y, z, roll, pitch, yaw]


def get_local_bbox(vehicle):
	'''This function returns the bounding box corners in a local frame'''
	dim_x = vehicle.bounding_box.extent.x
	dim_y = vehicle.bounding_box.extent.y
	dim_z = vehicle.bounding_box.extent.z
	
	c1 = [dim_x, dim_y, dim_z]
	c2 = [dim_x, -dim_y, dim_z]
	c3 = [dim_x, -dim_y, -dim_z]
	c4 = [dim_x, dim_y, -dim_z]

	c5 = [-dim_x, dim_y, dim_z]
	c6 = [-dim_x, -dim_y, dim_z]
	c7 = [-dim_x, -dim_y, -dim_z]
	c8 = [-dim_x, dim_y, -dim_z]
	
	# Adding center
	c0 = [0, 0, 0]
	
	return [c0, c1, c2, c3, c4, c5, c6, c7, c8]


def get_car_bbox_transform(vehicle):
	'''This function returns the transformation of the
	vehicle bounding box w.r.t world frame'''
	# Get the bounding box of the car in the car frame
	t_x = vehicle.bounding_box.location.x
	t_y = -vehicle.bounding_box.location.y
	t_z = vehicle.bounding_box.location.z
	
	# Transform to the car frame
	H_car_to_bbox = np.array([[1, 0, 0, t_x], [0, 1, 0, t_y], [0, 0, 1, t_z], [0, 0, 0, 1]])
	
	# Transform global
	x, y, z, roll, pitch, yaw = extract_actor_info(vehicle)
	H_W_to_car = make_transformation(x, y, z, roll, pitch, yaw)
	H_W_to_bbox = np.matmul(H_W_to_car, H_car_to_bbox)

	return H_W_to_bbox


def get_global_bbox(vehicle):
	'''This function returns the bounding box corners in a global frame'''
	H_W_to_bbox = get_car_bbox_transform(vehicle)
	local_bbox = np.asarray(get_local_bbox(vehicle)).T
	local_bbox = np.r_[local_bbox, [np.ones(9)]]
	global_bbox = np.matmul(H_W_to_bbox, local_bbox)
	
	# Returns numpy array 4x9
	return global_bbox


def get_bbox_ego_vehicle(ego_vehicle, vehicle):
	''' Gives bounding box corners w.r.t ego-vehicle'''
	H_W2Ego = get_car_bbox_transform(ego_vehicle)
	global_bbox = get_global_bbox(vehicle)
	ego_vehicle_bbox = np.matmul(np.linalg.pinv(H_W2Ego), global_bbox)

	return ego_vehicle_bbox


def get_radar_bbox(ego_vehicle_bbox, vehicle_to_radar_transform):
	'''This function tranforms the points in the ego-
	vehicle frame to the RADAR frame'''
	x, y, z, roll, pitch, yaw = vehicle_to_radar_transform
	# Transformation from ego to RADAR
	H_V_to_R = make_transformation(x, y, z, roll, pitch, yaw)
	# Bounding box in RADAR frame
	radar_bbox = np.matmul(np.linalg.pinv(H_V_to_R), ego_vehicle_bbox)

	return radar_bbox


def get_bounding_box(ego_vehicle, vehicle, vehicle_to_radar_transform):
	''' Get max vals of bounding box'''
	# NOTE: The return values are w.r.t to the RADAR
	# Carla does it opposite
	# -ve sign is to account for ROS convention
	# Find max y val and corresponding x val
	ego_vehicle_bbox = get_bbox_ego_vehicle(ego_vehicle, vehicle)
	# Get bounding box in RADAR frame
	radar_bbox = get_radar_bbox(ego_vehicle_bbox, vehicle_to_radar_transform)
	x = radar_bbox[0, 0]
	y = radar_bbox[1, 0]
	z = radar_bbox[2, 0]
	
	min_ind = np.argmin(radar_bbox[1, :])
	max_ind = np.argmax(radar_bbox[1, :])
	
	# Switching x and y
	z_max = radar_bbox[2, 0]
	y_max = radar_bbox[1, max_ind]
	x_max = radar_bbox[0, max_ind]
	
	# Similarly for min
	z_min = radar_bbox[2, 0]
	y_min = radar_bbox[1, min_ind]
	x_min = radar_bbox[0, min_ind]

	return [x, y, z, x_max, y_max, z_max, x_min, y_min, z_min]
