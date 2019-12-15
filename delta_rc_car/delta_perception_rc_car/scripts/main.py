#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Version : 1.0.0
Date    : Apr 07, 2019

References:
http://wiki.ros.org/message_filters
http://wiki.ros.org/cv_bridge/Tutorials/
http://docs.ros.org/api/image_geometry/html/python/
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscribe
'''

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division

# Handle paths and OpenCV import
from init_paths import *

# Built-in modules

# External modules
import matplotlib.pyplot as plt

# ROS modules
import tf
import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError

# ROS messages
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, CameraInfo
from delta_msgs.msg import (MarkerArrayStamped,
                            CameraTrack,
                            CameraTrackArray)

# Local python modules
from utils import *
from sort.sort import Sort
from darknet.darknet_video import YOLO
from ipm.ipm import InversePerspectiveMapping
from cube_marker_publisher import make_cuboid

# Global objects
STOP_FLAG = False
cmap = plt.get_cmap('tab10')
tf_listener = None

# Camera variables
CAMERA_INFO = None
CAMERA_EXTRINSICS = None
CAMERA_PROJECTION_MATRIX = None

# Frames
RADAR_FRAME = 'ti_mmwave'
EGO_VEHICLE_FRAME = 'rviz'
CAMERA_FRAME = 'rc_car/camera'

# Perception models
yolov3 = YOLO(configPath='cfg/yolov3-rc.cfg',
              weightPath='weights/yolov3-rc.weights',
              metaPath='cfg/rc-car_shoes.data')
ipm = InversePerspectiveMapping()
tracker = Sort(max_age=200, min_hits=1, use_dlib=False)

# FPS loggers
FRAME_COUNT = 0
all_fps = FPSLogger('Pipeline')
yolo_fps = FPSLogger('YOLOv3')
sort_fps = FPSLogger('Tracker')


########################### Functions ###########################


def camera_info_callback(camera_info):
    global CAMERA_INFO, CAMERA_PROJECTION_MATRIX
    if CAMERA_INFO is None:
        CAMERA_INFO = camera_info
        CAMERA_PROJECTION_MATRIX = np.matmul(np.asarray(CAMERA_INFO.P).reshape(3, 4), CAMERA_EXTRINSICS)


def visualize(img, tracked_targets, detections, publishers, timestamp, **kwargs):
    # Display tracked targets
    for tracked_target, detection in zip(tracked_targets, detections):
        label, score, bbox = detection
        x1, y1, x2, y2, tracker_id = tracked_target.astype('int')
        color = tuple(map(int, (np.asarray(cmap(tracker_id % 10))[:-1] * 255).astype('uint8')))
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.putText(img, '%s [%d%%] [ID: %d]' % (label.decode('utf-8').title(), score * 100, tracker_id),
            (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

    cv2_to_message(img, publishers['image_pub'], timestamp)


def publish_camera_tracks(publishers, tracked_targets, detections, timestamp):
    camera_track_array = CameraTrackArray()
    camera_track_array.header.frame_id = EGO_VEHICLE_FRAME
    camera_track_array.header.stamp = timestamp
    
    # Populate camera track message.
    for target, detection in zip(tracked_targets, detections):
        label, score, _ = detection
        u1, v1, u2, v2, track_id = target
        # todo(heethesh): Verify u/v convention here
        [x, y] = ipm.apply_homography(np.array([(u1 + u2) / 2, v2]))  # Bottom mid-point of bbox
        camera_track = CameraTrack()
        camera_track.x = y
        camera_track.y = -x
        camera_track.track_id = track_id
        camera_track.label = label
        camera_track.confidence = score
        camera_track_array.tracks.append(camera_track)

        # Publish camera track marker for debugging.
        camera_marker = make_cuboid(position=[y, -x, 0], scale=[0.2] * 3, # scale=[0.5, 0.25, 0.3],
            frame_id=EGO_VEHICLE_FRAME, marker_id=track_id,
            duration=0.5, color=[0, 0, 1], timestamp=timestamp)
        publishers['camera_marker_pub'].publish(camera_marker)

    # Publish the camera tarck data.
    publishers['tracker_pub'].publish(camera_track_array)


def roi_crop(img, size=[540, 720]):
    h, w, c = img.shape
    del_side = (w - size[1]) / 2
    del_top = h - size[0]
    cropped_img = img[int(del_top):, int(del_side):int(-del_side), :]
    return cropped_img


def perception_pipeline(img, timestamp, publishers, vis=True, **kwargs):
    # Log pipeline FPS
    all_fps.lap()

    # Preprocess
    # img = increase_brightness(img)
    img = roi_crop(img)

    # Object detection
    yolo_fps.lap()
    detections, frame_resized = yolov3.run(img)
    yolo_fps.tick()

    # Object tracking
    sort_fps.lap()
    dets = np.asarray([[bbox[0], bbox[1], bbox[2], bbox[3], score] for label, score, bbox in detections])
    tracked_targets = tracker.update(dets, img)
    sort_fps.tick()

    # Publish camera tracks
    publish_camera_tracks(publishers, tracked_targets, detections, timestamp)

    # Display FPS logger status
    all_fps.tick()
    # sys.stdout.write('\r%s | %s | %s ' % (all_fps.get_log(),
    #     yolo_fps.get_log(), sort_fps.get_log()))
    # sys.stdout.flush()

    # Visualize and publish image message
    if vis: visualize(img, tracked_targets, detections, publishers, timestamp)

    return detections


def perception_callback(image_msg, publishers, **kwargs):
    # Node stop has been requested
    if STOP_FLAG: return

    # Read image message
    img = message_to_cv2(image_msg)
    if img is None:
        print('Error')
        sys.exit(1)

    # Run the perception pipeline
    timestamp = image_msg.header.stamp
    detections = perception_pipeline(img.copy(), timestamp, publishers)


def shutdown_hook():
    global STOP_FLAG
    STOP_FLAG = True
    time.sleep(3)
    print('\n\033[95m' + '*' * 30 + ' Delta Perception Object Detection Shutdown ' + '*' * 30 + '\033[00m\n')


def run(**kwargs):
    global tf_listener, CAMERA_EXTRINSICS

    # Start node
    rospy.init_node('main', anonymous=True)
    rospy.loginfo('Current PID: [%d]' % os.getpid())
    tf_listener = tf.TransformListener()

    # Setup validation
    # validation_setup()
    
    # Setup models
    yolov3.setup()

    # Handle params and topics
    camera_info = rospy.get_param('~camera_info', '/camera/camera_info')
    image_color = rospy.get_param('~image_color', '/camera/image_color_rect')
    output_image = rospy.get_param('~output_image', '/delta/perception/object_detection/image')
    camera_track = rospy.get_param('~camera_track', '/delta/perception/ipm/camera_track')
    camera_track_marker = rospy.get_param('~camera_track_marker', '/delta/perception/camera_track_marker')

    # Display params and topics
    rospy.loginfo('CameraInfo topic: %s' % camera_info)
    rospy.loginfo('Image topic: %s' % image_color)
    rospy.loginfo('CameraTrackArray topic: %s' % camera_track)

    # Publish output topic
    publishers = {}
    publishers['image_pub'] = rospy.Publisher(output_image, Image, queue_size=5)
    publishers['tracker_pub'] = rospy.Publisher(camera_track, CameraTrackArray, queue_size=5)
    publishers['camera_marker_pub'] = rospy.Publisher(camera_track_marker, Marker, queue_size=5)

    # Subscribe to topics
    # info_sub = rospy.Subscriber(camera_info, CameraInfo, camera_info_callback)
    image_sub = message_filters.Subscriber(image_color, Image)

    # Synchronize the topics by time
    ats = message_filters.ApproximateTimeSynchronizer(
        [image_sub], queue_size=1, slop=0.5)
    ats.registerCallback(perception_callback, publishers, **kwargs)

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
