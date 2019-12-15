#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Version : 1.0.0
Date    : Apr 30, 2019

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

# ROS modules
import tf
import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError

# ROS messages
from sensor_msgs.msg import Image

# Local python modules
from utils import *
from ipm.ipm import InversePerspectiveMapping

# Frames
CAMERA_FRAME = 'ego_vehicle/camera/rgb/front'

# Perception models
ipm = InversePerspectiveMapping()

# FPS loggers
FRAME_COUNT = 0
ipm_fps = FPSLogger('IPM')


########################### Functions ###########################


def process(image_msg, image_pub, vis=True, **kwargs):
    # Read image message
    img = message_to_cv2(image_msg)

    # Run segmentation
    ipm_img = ipm.transform_image(img.copy())

    # Visualize and publish image message
    if vis: cv2_to_message(ipm_img, image_pub)


def callback(image_msg, image_pub, **kwargs):
    # Run the IPM pipeline
    process(image_msg, image_pub)


def shutdown_hook():
    print('\n\033[95m' + '*' * 30 + ' Inverse Perspective Mapping Shutdown ' + '*' * 30 + '\033[00m\n')


def run(**kwargs):
    # Start node
    rospy.init_node('main', anonymous=True)
    rospy.loginfo('Current PID: [%d]' % os.getpid())

    # Handle params and topics
    image_color = rospy.get_param('~image_color', '/camera/image_color_rect')
    output_image = rospy.get_param('~output_image', '/delta/perception_rc_car/ipm/image')

    # Display params and topics
    rospy.loginfo('Image topic: %s' % image_color)
    rospy.loginfo('Output topic: %s' % output_image)

    # Publish output topic
    image_pub = rospy.Publisher(output_image, Image, queue_size=5)

    # Subscribe to topics
    image_sub = message_filters.Subscriber(image_color, Image)

    # Synchronize the topics by time
    ats = message_filters.ApproximateTimeSynchronizer([image_sub], queue_size=1, slop=0.1)
    ats.registerCallback(callback, image_pub, **kwargs)

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
