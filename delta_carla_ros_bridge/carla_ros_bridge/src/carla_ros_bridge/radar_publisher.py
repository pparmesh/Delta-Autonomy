#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Prateek Parmeshwar
Email   : pparmesh@andrew.cmu.edu
Version : 1.0.0
Date    : Apr 08, 2019
'''

import time
import random
import traceback

import tf
import carla
import rospy
import numpy as np

from delta_msgs.msg import Track, TrackArray
from geometry_msgs.msg import Point, Polygon, Vector3
from radar_msgs.msg import RadarTrack, RadarTrackArray

from carla_ros_bridge.bridge import CarlaRosBridge
from carla_ros_bridge.bridge_with_rosbag import CarlaRosBridgeWithBag
from carla_ros_bridge.radar import simulate_radar, get_all_vehicles

RADAR_FRAME = '/ego_vehicle/radar'
VEHICLE_FRAME = '/ego_vehicle'


def list_to_ros_vector3(input_list):
    output_vector = Vector3()
    output_vector.x = input_list[0]
    output_vector.y = input_list[1]
    output_vector.z = input_list[2]
    return output_vector


def polygon_list_to_ros_points(vehicle):
    output_polygon = Polygon()
    
    mid_point = Point()
    mid_point.x, mid_point.y, mid_point.z = vehicle.x, vehicle.y, vehicle.z
    output_polygon.points.append(mid_point)

    max_point = Point()
    max_point.x, max_point.y, max_point.z = vehicle.x_max, vehicle.y_max, vehicle.z_max
    output_polygon.points.append(max_point)

    min_point = Point()
    min_point.x, min_point.y, min_point.z = vehicle.x_min, vehicle.y_min, vehicle.z_min
    output_polygon.points.append(min_point)

    return output_polygon


def publisher(actor_list, ego_vehicle):
    # Setup node
    pub = rospy.Publisher('/carla/ego_vehicle/radar/tracks', RadarTrackArray, queue_size=10)
    pub_ground_truth = rospy.Publisher('/carla/ego_vehicle/tracks/ground_truth', TrackArray, queue_size=10)

    # Define RADAR parameters
    theta_range = 2 * np.pi / 3
    dist_range = 150

    # Transform: [x, y, z, roll, pitch, yaw]
    radar_transform = [2.2, 0, 0.5, 0, 0, 0]
    ground_truth_transform = [0, 0, 0, 0, 0, 0]

    # Publish at a rate of 13Hz. This is the RADAR frequency
    rate = rospy.Rate(13)

    # Randomly publish some data
    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()
        br.sendTransform((radar_transform[0], radar_transform[1], radar_transform[2]), \
                      tf.transformations.quaternion_from_euler(radar_transform[3], radar_transform[4], radar_transform[5]), \
                      rospy.Time.now(), RADAR_FRAME, VEHICLE_FRAME)

        # Get list of all vehicles in Carla
        vehicles = get_all_vehicles(actor_list, ego_vehicle, radar_transform)

        # Get list of all detected vehicles
        ground_truth, radar_detections = simulate_radar(theta_range, dist_range, vehicles)

        # Publish RadarTrackArray
        radar_msg = RadarTrackArray()
        for detection in radar_detections:
            radar_track = RadarTrack()
            radar_track.track_id = detection.id
            radar_track.track_shape = polygon_list_to_ros_points(detection)
            radar_track.linear_velocity = list_to_ros_vector3(detection.velocity)
            radar_msg.tracks.append(radar_track)

        # Publish TrackArray
        ground_truth_msg = TrackArray()
        for ground_truth_vehicle in ground_truth:
            track = Track()
            track.label = 'vehicle'
            track.x = ground_truth_vehicle.x - radar_transform[0]
            track.y = ground_truth_vehicle.y - radar_transform[1]
            track.vx = ground_truth_vehicle.velocity[0]
            track.vy = ground_truth_vehicle.velocity[1]
            track.covariance = list(np.eye(4, dtype=np.float64).flatten())
            track.track_id = ground_truth_vehicle.id
            ground_truth_msg.tracks.append(track)

        # Header stamp and publish the RADAR message
        radar_msg.header.stamp = rospy.Time.now()
        radar_msg.header.frame_id = RADAR_FRAME
        pub.publish(radar_msg)

        # Header stamp and publish the ground truth message
        ground_truth_msg.header.stamp = rospy.Time.now()
        ground_truth_msg.header.frame_id = VEHICLE_FRAME
        pub_ground_truth.publish(ground_truth_msg)
        
        rate.sleep()


def main():
    '''
    main function for carla simulator ROS bridge
    maintaining the communication client and the CarlaRosBridge objects
    '''
    rospy.init_node('radar_client', anonymous=True)
    params = rospy.get_param('carla')
    host = params['host']
    port = params['port']

    rospy.loginfo('Radar trying to connect to {host}:{port}'.format(host=host, port=port))

    try:
        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(2000)
        carla_world = carla_client.get_world()
        rospy.logwarn('Radar Connected')

        ego_vehicle = None
        while ego_vehicle is None:
            # Get all  vehicle actors in environment
            actor_list = carla_world.get_actors().filter('vehicle.*')
            npc_list = []

            # Get ego vehicle object and remove it from actor list
            for actor in actor_list:
                attribute_val = actor.attributes['role_name']
                if attribute_val == 'ego_vehicle':
                    ego_vehicle = actor
                else:
                    npc_list.append(actor)

            if ego_vehicle is not None: break
            else:
                rospy.logwarn('Ego vehicle not found, will keep trying after 1s...')
                time.sleep(1)

        rospy.logwarn('Radar started publishing')
        publisher(npc_list, ego_vehicle)
        
        rospy.loginfo('Radar delete world and client')
        del carla_world
        del carla_client

    except Exception as error:
        rospy.logerr('RADAR Error {}'.format(error))
        traceback.print_exc()

    finally:
        rospy.logwarn('RADAR Node Exiting')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print('ROSInterruptException occurred')
