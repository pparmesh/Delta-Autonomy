# This script mimics a RADAR sensor. It request all its transformations from the ClientRADAR script.
# Refer that script for all transformations and bounding box values of all vehicles in world or ego
# vehicle frame

from __future__ import division

import copy
import numpy as np
import matplotlib.pyplot as plt

from radar_utils import *


class RadarParams:
    ''' This class instantiates a RADAR object'''
    def __init__(self, theta, radius):
        ''' theta is the angular range of the RADAR.
        range is the distance up to which RADAR detects objects'''
        self.theta = theta
        self.r = radius


class Vehicle:
    ''' instantiates a vehicle object. It has x,y of centroid and
    bounding box with points (x_max, y_max) and (x_min,y_min)'''

    # NOTE: y_max is not actually max y co-ordinate and similarly for y_min
    def __init__(self, actor_id, bbox, velocity):
        self.id = actor_id
        self.x = bbox[0]
        self.y = bbox[1]
        self.z = bbox[2]
        self.x_max = bbox[3]
        self.y_max = bbox[4]
        self.z_max = bbox[5]
        self.x_min = bbox[6]
        self.y_min = bbox[7]
        self.z_min = bbox[8]
        self.velocity = velocity


def field_of_view_filter(vehicles, radar):
    '''Given a list of vehicles in the environment. This function filters all
    those vehicles that are not in the field of view(FOV) of the RADAR.
    It returns a list of vehicles (x,y,yaw) of all vehicles in FOV'''
    # Iterate over all y values of vehicles
    filtered_vehicles = []
    for vehicle in vehicles:
        # Take the left most point of the vehicle and take abs
        x = vehicle.x_min
        y = vehicle.y_min
        if x >= 1 / np.tan(radar.theta / 2) * abs(y) and x <= radar.r and x > 0:
            filtered_vehicles.append(vehicle)
    return filtered_vehicles


def noise_function(x_pos, radar, model='linear'):
    '''
    Helper function to add RADAR based on some function
    Returns RADAR noise based on distance of object from RADAR
    '''
    scaling_factor = radar.r
    # Linear function
    if model == 'linear':
        temp = 3 * x_pos # std_dev cannot be negative
        noise = np.random.normal(0, np.abs(temp)) / scaling_factor
        return noise
    # Quadratic Function
    elif model == 'quadratic':
        temp = 2 * x_pos * x_pos # std_dev cannot be negative
        noise = np.random.normal(0, np.abs(temp)) / (scaling_factor * scaling_factor)
        return noise
    else:
        raise Exception('Noise model not supported')


def add_radar_noise(detected_vehicles, radar, model,
    dropout_prob=0.15, ghost_prob=0.3):
    detected_vehicles_noisy = []
    for vehicle in detected_vehicles:
        # Add noise on detections (position and velocity)
        noise = noise_function(vehicle.x, radar, 'linear')
        vehicle.y_max += noise / 3
        vehicle.y_min += noise / 3
        vehicle.y += noise / 3
        vehicle.x_max += noise
        vehicle.x_min += noise
        vehicle.x += noise
        vehicle.velocity[0] += noise / 3
        vehicle.velocity[1] += noise / 5

        # Dropouts with certain probability
        prob = np.random.rand()
        if prob > dropout_prob:
            detected_vehicles_noisy.append(vehicle)

    # Add ghost detections (false positives) with certain probability
    prob = np.random.rand()
    if prob < ghost_prob:
        # Generate random data
        random_track_id = 65000
        random_x = np.random.rand() * 95 + 15  # X-axis range: +015m to +110m
        random_y = np.random.rand() * 30 - 15  # Y-axis range: -015m to +015m
        random_bbox = np.array([[random_x, random_y, 0.0] for _ in range(3)]).flatten()
        random_velocity = np.r_[np.random.normal() * 0.25, np.random.normal() * 0.1, 0.0]
        
        # Create ghost vehicle and filter within FOV
        ghost_vehicle = Vehicle(random_track_id, random_bbox, random_velocity)
        ghost_vehicle = field_of_view_filter([ghost_vehicle], radar)[0]
        detected_vehicles_noisy.append(ghost_vehicle)

    return detected_vehicles_noisy


def detect_vehicles_fov(vehicles, radar):
    '''Returns list of vehicle objects detected by RADAR'''
    fov_vehicles = field_of_view_filter(vehicles, radar)
    if len(fov_vehicles) == 0: return []
    
    # Sort vehicles based on x values
    fov_vehicles = sorted(fov_vehicles, key=lambda x: x.x)
    
    # For every detected vehicle my RADAR will not be able to detect the
    # vehicles other detected vehicles are blocking. Every equation will have a slope
    slopes = []
    detected_vehicles = []
    flag = False
    
    # Iterate through FOV vehicles already sorted based on x
    for vehicle in fov_vehicles:
        x_max = vehicle.x_max
        x_min = vehicle.x_min
        y_max = vehicle.y_max
        y_min = vehicle.y_min
        
        # Find slopes of the detected vehicle
        m1 = np.tan(x_max / y_max)
        m2 = np.tan(x_min / y_min)
        if vehicle == fov_vehicles[0]:
            slopes.append([m1, m2])
            detected_vehicles.append(vehicle)
        else:
            for m in slopes:
                if m1 < m[0] or m2 > m[1]: flag = True
                else: flag = False
        if flag:
            slopes.append([m1, m2])
            detected_vehicles.append(vehicle)

    return detected_vehicles


def parse_velocity(ego_vehicle, vehicle):
    '''Returns velocity in ego vehicle frame'''
    H_W_to_ego = get_car_bbox_transform(ego_vehicle)
    x_vel = vehicle.get_velocity().x
    y_vel = -vehicle.get_velocity().y
    z_vel = vehicle.get_velocity().z
    # The above values are in the world coordinate frame
    # Transforming them w.r.t ego vehicle frame
    vel_vec = np.array([x_vel, y_vel, z_vel, 0])
    vel_ego = np.matmul(np.linalg.pinv(H_W_to_ego), vel_vec)

    return vel_ego


def get_all_vehicles(actor_list, ego_vehicle, transform):
    vehicles = []
    for actor in actor_list:
        velocity_in_ego_frame = parse_velocity(ego_vehicle, actor)
        bbox = get_bounding_box(ego_vehicle, actor, transform)
        vehicles.append(Vehicle(actor.id, bbox, velocity_in_ego_frame))
    return vehicles


def simulate_radar(theta, radius, vehicles):
    '''Simulate and visualize RADAR output'''
    radar = RadarParams(theta, radius)
    detected_vehicles = detect_vehicles_fov(vehicles, radar)
    detected_vehicles_noise = add_radar_noise(copy.deepcopy(detected_vehicles), radar, model='linear')
    return detected_vehicles, detected_vehicles_noise


def visualize_radar(vehicles, detected_vehicles, radar, ego_vehicle):
    '''This function shows a scatter plot of vehicles in environment,
    the RADAR detected vehicles, and the ego vehicle'''
    env_plot = [[vehicle.x, vehicle.y] for vehicle in vehicles]
    env_plot = np.asarray(env_plot)

    det_plot = [[vehicle.x, vehicle.y] for vehicle in detected_vehicles]
    det_plot = np.asarray(det_plot)

    fig = plt.figure()
    ax1 = fig.add_subplot(111)

    ax1.scatter(env_plot[:, 0], env_plot[:, 1], s=10, c='b', marker='s', label='Carla Vehicles')
    if len(detected_vehicles) != 0:
        ax1.scatter(det_plot[:, 0], det_plot[:, 1], s=10, c='r', marker='o', label='Detected Vehicles')
    ax1.scatter(ego_vehicle.x, ego_vehicle.y, s=15, c='g', marker='+', label='Ego Vehicle')
    
    # Plot RADAR FOV
    xlim = np.tan(radar.theta / 2) * radar.r
    x = np.linspace(-xlim, xlim, 100)
    y = 1 / np.tan(radar.theta / 2) * abs(x)
    ax1.plot(x, y, '-r', label='FOV')
    
    # Set limits for plot
    ax1.set_xlim(-100, 100)
    ax1.set_ylim(-500, 500)
    plt.legend(loc='upper left')
    plt.show()
