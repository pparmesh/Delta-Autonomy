#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Version : 1.0.0
Date    : Nov 24, 2019
'''

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division

# Built-in modules
import os
import sys
import time
import os.path as osp

# External modules
import motmetrics as mot
import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(precision=3, suppress=True, threshold=sys.maxsize)

# ROS modules
import rospy
import tf2_ros
import message_filters

# ROS messages
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from delta_msgs.msg import Track, TrackArray

# Global objects
STOP_FLAG = False

# Frames
RADAR_FRAME = 'ti_mmwave'
EGO_VEHICLE_FRAME = 'rviz'


########################### Functions ###########################


def convert_tracks_to_numpy(msg):   
    tracks = [[track.x, track.y, track.vx, track.vy] for track in msg.tracks]
    return np.array(tracks)


def convert_pose_to_numpy(msg):
    pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    return np.array(pose)


def find_nearest_pose(src, dst):
    dists = np.sum((src[:, :2] - dst[:2]) ** 2, axis=1)
    return src[np.argmin(dists)]


########################### Classes ###########################


class AveragingFilter: 
    def __init__: 
        self.vx_history = []
        self.vy_history = []
    
    def update(self, velocity):
        vx, vy, vz = velocity
        window_size = 10
        if (len(self.vx_history) == window_size):
            self.vx_history.remove(self.vx_history[0])
            self.vy_history.remove(self.vy_history[0])

        self.vx_history.append(vx)
        self.vy_history.append(vy)

        averaged_vx = np.mean(self.vx_history)
        averaged_vy = np.mean(self.vy_history)
        return np.array([averaged_vx, averaged_vy, 0])


class OptiTrackData:
    def __init__(self, frame, tf_buffer):
        self.frame = frame
        self.pose = None
        self.velocity = None
        self.timestamp = None
        self.first_time = True
        self.tf_buffer = tf_buffer
        self.filter = AveragingFilter()

    @property
    def state(self):
        if self.first_time: return None
        return np.r_[self.pose[:2], self.velocity[:2]]

    def transform_frame(self, pose, target_frame='rviz'):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, pose.header.frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

        return tf2_geometry_msgs.do_transform_pose(pose, transform)

    def update(self, pose_msg):
        pose_msg_trans = self.transform_frame(pose_msg)
        if pose_msg_trans is None: return
        pose = convert_pose_to_numpy(pose_msg_trans)
        timestamp = pose_msg.header.stamp

        if not self.first_time:
            self.estimate_velocity(pose, timestamp)

        self.pose = pose
        self.timestamp = timestamp
        self.first_time = False

    def estimate_velocity(self, pose, timestamp):
        dx = pose - self.pose
        dt = timestamp - self.timestamp
        if dt.to_sec() < 0.009: return
        self.velocity = dx / dt.to_sec()
        self.velocity = self.filter.update(self.velocity)


class OptiTrackValidation:
    def __init__(self):
        self.position_errors = []
        self.velocity_errors = []
        self.position_gt = []
        self.velocity_gt = []
        self.position_det = []
        self.velocity_det = []
        self.velocity_success = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.acc = mot.MOTAccumulator(auto_id=True)
        self.rc_car = OptiTrackData('rc_car', self.tf_buffer)
        self.oncoming_car = OptiTrackData('oncoming_car', self.tf_buffer)

    def validate_iou(self, detection, ground_truth, bbox_dim=[2.0, 2.0], max_iou=0.5):
        # Compute cost matrix.
        cost_matrix = mot.distances.iou_matrix(np.array([np.r_[ground_truth, bbox_dim]]),
            np.array([np.r_[detection, bbox_dim]]), max_iou=max_iou)

        # Accumulate data for validation.
        self.acc.update([0], [0], cost_matrix)

    def tracks_callback(self, tracks_msg, verbose=False):
        # Node stop has been requested
        if STOP_FLAG: return

        # Associate nearest detections with the ground truth pose
        tracks = convert_tracks_to_numpy(tracks_msg)
        oncoming_car = self.oncoming_car.state
        if oncoming_car[0] > 4.0: return
        if oncoming_car is None or not len(tracks): return
        detected_car = find_nearest_pose(tracks, oncoming_car)

        tolerance = 0.2
        min_vel = abs(detected_car[2] * (1.0 - tolerance))
        max_vel = abs(detected_car[2] * (1.0 + tolerance))
        self.velocity_success.append(1.0 if min_vel < abs(oncoming_car[2]) and \
            oncoming_car[2] < max_vel else 0.0)

        if verbose:
            print('\nDetection:', detected_car)
            print('Ground Truth:', oncoming_car)
            print(np.abs([min_vel, oncoming_car[2], max_vel]))

        # Store errors
        errors = (detected_car - self.oncoming_car.state) ** 2
        self.position_errors.append(np.sqrt(np.sum(errors[:2])))
        self.velocity_errors.append(np.sqrt(np.sum(errors[2])))
        self.position_gt.append(oncoming_car[:2])
        self.velocity_gt.append(oncoming_car[2:])
        self.position_det.append(detected_car[:2])
        self.velocity_det.append(detected_car[2:])

        # MOT metrics
        self.validate_iou(detected_car[:2], self.oncoming_car.state[:2])

    def plot(self):
        self.position_gt = np.array(self.position_gt)
        self.velocity_gt = np.array(self.velocity_gt)
        self.position_det = np.array(self.position_det)
        self.velocity_det = np.array(self.velocity_det)

        plt.figure()
        plt.title('Position (m)')
        plt.xlim(-3, 3)
        plt.ylim(-0.5, 5)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.plot(-self.position_gt[:, 1], self.position_gt[:, 0], label='Ground Truth')
        plt.plot(-self.position_det[:, 1], self.position_det[:, 0], label='Nearest Detection')
        plt.grid()
        plt.legend()
        filename = osp.join(osp.dirname(osp.realpath(__file__)), 'position_plot.png')
        plt.savefig(filename, dpi=300)

        plt.figure()
        plt.title('Velocity X (m/s)')
        plt.plot(self.velocity_gt[:, 0], label='Ground Truth')
        plt.plot(self.velocity_det[:, 0], label='Nearest Detection')
        plt.grid()
        plt.legend()
        filename = osp.join(osp.dirname(osp.realpath(__file__)), 'vx_plot.png')
        plt.savefig(filename, dpi=300)

        plt.figure()
        plt.title('Velocity Y (m/s)')
        plt.plot(self.velocity_gt[:, 1], label='Ground Truth')
        plt.plot(self.velocity_det[:, 1], label='Nearest Detection')
        plt.grid()
        plt.legend()
        filename = osp.join(osp.dirname(osp.realpath(__file__)), 'vy_plot.png')
        # plt.savefig(filename, dpi=300)

    def shutdown_hook(self):
        global STOP_FLAG
        STOP_FLAG = True
        time.sleep(3)

        print('\n\033[95m' + '*' * 25 + ' Delta OptiTrack Validation Results ' + '*' * 25 + '\033[00m\n')
        print('Position RMSE: %.3f m' % np.mean(self.position_errors))
        print('Velocity RMSE: %.3f m/s' % np.mean(self.velocity_errors))
        # print('Velocity Accuracy: %.2f%%' % (np.mean(self.velocity_success) * 100))
        self.plot()

        metrics = mot.metrics.create()
        summary = metrics.compute(self.acc, metrics=mot.metrics.motchallenge_metrics, name='Overall')
        print(mot.io.render_summary(summary, formatters=metrics.formatters,
            namemap=mot.io.motchallenge_metric_names), '\n')


def run(**kwargs):
    # Start node
    rospy.init_node('optitrack_validation', anonymous=False)
    rospy.loginfo('Current PID: [%d]' % os.getpid())

    # Handle params and topics
    tracks = rospy.get_param('~tracks', '/delta/tracking_fusion/tracker/tracks')
    rc_car_pose = rospy.get_param('~rc_car_pose', '/vrpn_client_node/rc_car/pose')
    oncoming_pose = rospy.get_param('~oncoming_pose', '/vrpn_client_node/oncoming_car/pose')

    # Display params and topics
    rospy.loginfo('TrackArray topic: %s' % tracks)
    rospy.loginfo('RC Car Pose topic: %s' % rc_car_pose)
    rospy.loginfo('Oncoming Car Pose topic: %s' % oncoming_pose)

    # OptiTrack validator
    validator = OptiTrackValidation()

    # Subscribe to topics
    rospy.Subscriber(tracks, TrackArray, validator.tracks_callback)
    # rospy.Subscriber(rc_car_pose, PoseStamped, validator.rc_car.update)
    rospy.Subscriber(oncoming_pose, PoseStamped, validator.oncoming_car.update)

    # Shutdown hook
    rospy.on_shutdown(validator.shutdown_hook)

    # Keep python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')


if __name__ == '__main__':
    # Start OptiTrack validation node
    run()
