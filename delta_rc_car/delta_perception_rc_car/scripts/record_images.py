#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
To run the node:
chmod +x record_images.py
rosrun delta_rc_car record_images

To debug and view image:
rosrun image_view image_view image:=/delta/rc_car/recorder/image
'''

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division

# Handle paths and OpenCV import
from init_paths import *

from datetime import datetime

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageRecorder:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/delta/rc_car/recorder/image', Image)
        self.image_sub = rospy.Subscriber('image_topic', Image, self.callback)
        self.timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.count = 0

    def callback(self, data):
        try: cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e: return

        h, w, c = cv_image.shape
        cropped_image = cv_image[int(0.4 * h):, int(0.25 * w):int(0.75 * w), :]

        folder = os.path.join(os.path.expanduser('~'), 'data_collection/' + self.timestamp)
        if not os.path.exists(folder): os.makedirs(folder)
        filename = '%s/%s-%05d.jpg' % (folder, self.timestamp, self.count)
        cv2.imwrite(filename, cropped_image)
        self.count += 1

        try: self.image_pub.publish(self.bridge.cv2_to_imgmsg(cropped_image, 'bgr8'))
        except CvBridgeError as e: return


def main(args):
    ic = ImageRecorder()
    rospy.init_node('image_recorder', anonymous=True)
    try: rospy.spin()
    except KeyboardInterrupt: print('Shutting down')
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
