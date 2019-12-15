#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Author: Heethesh Vhavle
# @Date:   Nov 20, 2019
# @Last Modified by:   Heethesh Vhavle
# @Last Modified time: Nov 21, 2019

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division

# Handle paths and OpenCV import
from init_paths import *

# ROS modules
import tf
import rospy
import ros_numpy
import message_filters
import sensor_msgs.point_cloud2 as pcl2

# External modules
import matplotlib.pyplot as plt
from pyclustering.cluster.dbscan import dbscan

# ROS messages
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from ti_mmwave_rospkg.msg import RadarScan, RadarScanArray
from radar_msgs.msg import RadarDetection, RadarDetectionArray

# Local python modules
from utils import *
from cube_marker_publisher import make_cuboid

# Global objects
STOP_FLAG = False
tf_listener = None

# Frames
RADAR_FRAME = 'ti_mmwave'
EGO_VEHICLE_FRAME = 'rc_car'

# FPS loggers
logger = FPSLogger('Radar Pipeline')


########################### Functions ###########################


def make_point_cloud2(targets, header):
    points = np.c_[targets[:, 0], targets[:, 1], np.zeros_like(x)]
    return pcl2.create_cloud_xyz32(header, points)


def publish_radar_messages(targets, header, publishers):
    detections_array = RadarDetectionArray()
    detections_array.header = header

    for i, target in enumerate(targets):
        detection = RadarDetection()
        detection.detection_id = i
        detection.position = Point()
        detection.position.x = target[0]
        detection.position.y = target[1]
        detection.velocity = Vector3()
        detection.velocity.x = target[2]
        detection.velocity.y = target[3]
        detection.amplitude = target[4]

        # Filter detections
        if target[5] < 5 and np.abs(target[7]) > 0:
            detections_array.detections.append(detection)

        # Visualization marker
        radar_marker = make_cuboid(position=[target[0], target[1], 0], scale=[0.2] * 3,
            frame_id=RADAR_FRAME, marker_id=i, duration=0.1,
            color=[1, 0, 0], timestamp=header.stamp)
        publishers['radar_marker_pub'].publish(radar_marker)

    publishers['radar_dets_pub'].publish(detections_array)
    # radar_pcl = make_point_cloud2(targets, header)
    # publishers['radar_pcl_pub'].publish(radar_pcl)


def radar_detector(radar_msg, publishers):
    # Convert radar scan array to numpy array
    states = np.array([[scan.x, scan.y, scan.velocity, scan.range, scan.bearing, scan.intensity]
                        for scan in radar_msg.scans if scan.range < 5])

    # Extract states
    x = states[:, 0]
    y = states[:, 1]
    r = states[:, 3]
    th = np.deg2rad(states[:, 4])
    vr = states[:, 2]
    vx = vr * np.cos(th)
    vy = vr * np.sin(th)
    db = states[:, 5]

    # Clustering
    features = np.c_[x, y, vx, vy, db, r, th, vr]
    dbscan_instance = dbscan(features[:, 5:], 0.7, 3)
    dbscan_instance.process()
    clusters = dbscan_instance.get_clusters()
    targets = np.array([np.mean(features[cluster], axis=0) for cluster in clusters])

    # Publish messages
    publish_radar_messages(targets, radar_msg.header, publishers)


def callback(radar_msg, publishers, **kwargs):
    # Node stop has been requested
    if STOP_FLAG: return

    # Log pipeline FPS
    logger.lap()
    radar_detector(radar_msg, publishers)
    logger.tick()

    # Display FPS logger status
    # sys.stdout.write('\r%s ' % (logger.get_log()))
    # sys.stdout.flush()


def shutdown_hook():
    global STOP_FLAG
    STOP_FLAG = True
    time.sleep(3)
    print('\n\033[95m' + '*' * 30 + ' Delta Perception Radar Shutdown ' + '*' * 30 + '\033[00m\n')


def run(**kwargs):
    global tf_listener

    # Start node
    rospy.init_node('main', anonymous=True)
    rospy.loginfo('Current PID: [%d]' % os.getpid())
    tf_listener = tf.TransformListener()

    # Handle params and topics
    radar = rospy.get_param('~radar', '/ti_mmwave/radar_scan_array')
    radar_pcl = rospy.get_param('~radar_pcl', '/delta/perception/radar/point_cloud')
    radar_dets = rospy.get_param('~radar_dets', '/delta/perception/radar/detections')
    radar_marker = rospy.get_param('~radar_marker', '/delta/perception/radar/marker')

    # Display params and topics
    rospy.loginfo('RADAR topic: %s' % radar)

    # Publish output topic
    publishers = {}
    publishers['radar_pcl_pub'] = rospy.Publisher(radar_pcl, PointCloud2, queue_size=5)
    publishers['radar_marker_pub'] = rospy.Publisher(radar_marker, Marker, queue_size=5)
    publishers['radar_dets_pub'] = rospy.Publisher(radar_dets, RadarDetectionArray, queue_size=5)

    # Subscribe to topics
    radar_sub = message_filters.Subscriber(radar, RadarScanArray)

    # Synchronize the topics by time
    ats = message_filters.ApproximateTimeSynchronizer(
        [radar_sub], queue_size=1, slop=0.5)
    ats.registerCallback(callback, publishers, **kwargs)

    # Shutdown hook
    rospy.on_shutdown(shutdown_hook)

    # Keep python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')


if __name__ == '__main__':
    # Start perception node
    run()
