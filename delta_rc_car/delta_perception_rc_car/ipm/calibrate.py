# -*- coding: utf-8 -*-

__author__ = "Heethesh Vhavle"
__email__ = "heethesh@cmu.edu"
__version__ = "1.0.0"

import os
import sys
import yaml

ROS_CV = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ROS_CV in sys.path: sys.path.remove(ROS_CV)

import cv2
import numpy as np
import matplotlib.pyplot as plt
from transformations import euler_matrix, translation_matrix


class InversePerspectiveMapping:
    def __init__(self, filename):
        self.info = self.load_camera_info(filename)
        self.frame_width = self.info['width']
        self.frame_height = self.info['height']

        # Intrinsics
        self.K = np.zeros((3, 4))
        self.K[:, :3] = np.asarray(self.info['intrinsics']['K']).reshape(3, 3)
        self.D = np.asarray(self.info['intrinsics']['D'])

        # Extrinsics
        self.extrinsics_euler = np.radians(self.info['extrinsics']['rpy'])
        self.extrinsics_rotation = euler_matrix(self.extrinsics_euler[0],
            self.extrinsics_euler[1], self.extrinsics_euler[2])
        self.extrinsics_translation = translation_matrix(self.info['extrinsics']['position'])

        # Homography calibration parameters
        self.ipm_matrix = None
        self.in_points = np.float32([(479, 407), (816, 408), (316, 678), (980, 682)])  # Checkerboard corners
        self.homography_dims = (500, 500)
        self.homography_image_dims = (2500, 2000)
        self.homography_base = 1040
        self.homography_translation = np.asarray([[0, 0, 0], [0, 0, -1800], [0, 0, 0]])

        # Metric calibration parameters
        self.metric_scale_factor = (0.09, 0.435)
        self.metric_rotation_yaw = -0.1
        self.checkerboard_center = (115, 827)  # Checkerboard center after homography
        self.calibration_ego_y = -3.946949
        self.calibration_grid_px = 44
        self.calibration_grid_m = 0.4 * 8  # 40cm, 8x8
        self.ipm_px_to_m = self.calibration_grid_m / self.calibration_grid_px
        self.ipm_m_to_px = self.calibration_grid_px / self.calibration_grid_m

        # Overwrite calibration data if available
        if 'calibration' not in self.info:
            print('WARNING: Calibration not performed, run InversePerspectiveMapping.calibrate() first!')
            return
        else:
            print('Loading calibration data from file...')
            self.ipm_matrix = np.asarray(self.info['calibration']['ipm_matrix']).reshape(3, 3)
            self.ipm_image_dims = tuple(self.info['calibration']['ipm_image_dims'][::-1])
            self.ipm_px_to_m = self.info['calibration']['ipm_px_to_m']
            self.ipm_m_to_px = self.info['calibration']['ipm_m_to_px']
            self.calibration_ego_y = self.info['calibration']['calibration_ego_y']

    def load_camera_info(self, filename):
        with open(filename, 'r') as f:
            camera_info = yaml.load(f)
        return camera_info

    def transform_image(self, img):
        img = cv2.warpPerspective(img, self.ipm_matrix, self.ipm_image_dims)
        return img

    def transform_points_to_px(self, points, squeeze=True):
        ones = np.ones((len(points), 1))
        if len(np.array(points).shape) == 1:
            points = np.expand_dims(points, axis=0)
            ones = np.array([[1]])

        points_px = self.ipm_matrix @ np.hstack([points, ones]).T
        points_px = points_px / points_px[-1]

        if squeeze: return points_px.T[:, :2].squeeze()
        return points_px.T[:, :2]

    def transform_points_to_m(self, points):
        points_px = self.transform_points_to_px(points, squeeze=False)
        points_px[:, 0] = points_px[:, 0] - self.ipm_image_dims[0] / 2
        points_px[:, 1] = self.ipm_image_dims[1] - points_px[:, 1]

        points_m = points_px * self.ipm_px_to_m
        points_m[:, 1] += self.calibration_ego_y
        return points_m.squeeze()

    def calibrate(self, frame):
        # Perspective mapping
        b, h, w = self.homography_base, self.homography_dims[0], self.homography_dims[1]
        out_points = np.float32([(b, b), (h + b, b), (b, w + b), (h + b, w + b)])
        homography = cv2.getPerspectiveTransform(self.in_points, out_points)
        homography += self.homography_translation
        dst = cv2.warpPerspective(frame, homography, self.homography_image_dims)

        # Metric scale rectification
        dst = cv2.resize(dst, (0, 0), fx=self.metric_scale_factor[0], fy=self.metric_scale_factor[1])
        scale = np.asarray([[self.metric_scale_factor[0], 0, 0], [0, self.metric_scale_factor[1], 0], [0, 0, 1]])

        # Metric rotation rectification
        rotation = cv2.getRotationMatrix2D((dst.shape[1] / 2, dst.shape[0] / 2), self.metric_rotation_yaw, 1)
        dst = cv2.warpAffine(dst, rotation, (dst.shape[1], dst.shape[0]))
        rotation = np.vstack([rotation, [0, 0, 1]])

        # Metric offset rectification
        board_center_px = self.calibration_ego_y * self.ipm_m_to_px
        offset_translation = np.asarray([[1, 0, -(self.checkerboard_center[0] - dst.shape[1] / 2)],
                                         [0, 1, -(self.checkerboard_center[1] - dst.shape[0] - board_center_px)],
                                         [0, 0, 1]])
        dst = cv2.warpPerspective(dst, offset_translation, (dst.shape[1], dst.shape[0]))

        # Overall IPM matrix
        self.ipm_matrix = offset_translation @ rotation @ scale @ homography

        return dst, self.ipm_matrix

    def find_homography(self, frame, in_points, out_points):
        self.ipm_matrix = cv2.findHomography(in_points, out_points)[0]
        dst = cv2.warpPerspective(frame, self.ipm_matrix, self.homography_image_dims)
        return dst, self.ipm_matrix

    def apply_homography(self, point):
        point = np.r_[point, 1]
        out_point = self.ipm_matrix @ point
        out_point = out_point / out_point[-1]
        return out_point


def roi_crop(img, size=[540, 720]):
    h, w, c = img.shape
    del_side = (w-size[1])/2
    del_top = h- size[0]
    cropped_img = img[int(del_top):, int(del_side):int(-del_side), :]
    return cropped_img


def pick_points(img):
    points = []

    def draw_circle(event, x, y, flags, param):
        nonlocal points, img
        if event == cv2.EVENT_LBUTTONDBLCLK:
            cv2.circle(img, (x, y), 5, (255, 0, 0), -1)
            points.append([x, y])
            print(points[-1])

    win_name = 'Picker'
    cv2.namedWindow(win_name)
    cv2.setMouseCallback(win_name, draw_circle)
    cv2.moveWindow(win_name, 100, 100)

    while True:
        cv2.imshow(win_name, img)
        k = cv2.waitKey(20) & 0xFF
        if k == 27: break

    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return np.array(points)


def perform_calibration(config_file, image_file):
    ipm = InversePerspectiveMapping(config_file)

    win_name = 'Calibration'
    cv2.namedWindow(win_name)
    cv2.moveWindow(win_name, 100, 100)

    image = cv2.imread(image_file)
    image = roi_crop(image)
    # output, ipm_matrix = ipm.calibrate(image)
    output, ipm_matrix = ipm.find_homography(image)

    # Display calibration results
    print('ipm_matrix:', ipm_matrix.flatten())
    # print('ipm_image_dims:', list(output.shape)[:2])
    # print('ipm_px_to_m:', ipm.ipm_px_to_m)
    # print('ipm_m_to_px:', ipm.ipm_m_to_px)
    # print('calibration_ego_y:', ipm.calibration_ego_y)

    in_points = np.array([[385, 411], [716, 193], [391, 148], [-15, 199]])
    # out_points = np.array([[0.36, 0], [0.56, 1.03], [1.48, 0], [-0.56, 0.8]])
    for point in in_points:
        print(ipm.apply_homography(point))

    # cv2.imshow(win_name, output)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

def reproject(ipm_matrix, image_points, world_points):
    points = np.c_[world_points, np.ones((len(world_points), 1))]
    points = (np.linalg.inv(ipm_matrix) @ points.T).T
    points = (points / points[:, -1].reshape(-1, 1))[:, :2]
    error = np.sqrt(np.mean((points - image_points) ** 2))
    return points, error


def reproject_world(ipm_matrix, image_points, world_points):
    points = np.c_[image_points, np.ones((len(image_points), 1))]
    points = (ipm_matrix @ points.T).T
    points = (points / points[:, -1].reshape(-1, 1))[:, :2]
    error = np.sqrt(np.mean((points - world_points) ** 2))
    return points, error


def find_homography_ransac(ipm, image, image_points, world_points):
    best_matrix = None
    best_error = 100000

    weights = np.hstack([np.array([0.5] * 41) / 41, np.array([0.3] * 47) / 47, np.array([0.2] * 21) / 21])

    for i in range(100):
        idx = np.random.choice(len(world_points), 50, replace=False, p=weights)
        output, ipm_matrix = ipm.find_homography(image, image_points[idx], world_points[idx])
        _, error1 = reproject(ipm_matrix, image_points, world_points)
        _, error2 = reproject_world(ipm_matrix, image_points, world_points)
        # print(error1, error2)
        error = error1

        if error < best_error:
            best_error = error
            best_matrix = ipm_matrix

    print(best_error)
    return best_matrix


def display_reprojection(image, ipm_matrix, image_points, world_points, world=False):
    if world:
        points, error = reproject_world(ipm_matrix, image_points, world_points)
        org_points = world_points
    else:
        points, error = reproject(ipm_matrix, image_points, world_points)
        org_points = image_points

    if not world: plt.imshow(image)
    plt.scatter(org_points[:, 0], org_points[:, 1], c='g')
    plt.scatter(points[:, 0], points[:, 1], c='r')
    plt.grid()
    plt.show()


def perform_calibration2(config_file, image_file):
    ipm = InversePerspectiveMapping(config_file)
    image = cv2.imread(image_file)
    image = roi_crop(image)
    # points = pick_points(image)

    from world_points import world_points
    from image_points import image_points

    world_points, image_points = np.vstack(world_points), np.vstack(image_points)
    ipm_matrix = find_homography_ransac(ipm, image, image_points, world_points)

    display_reprojection(image, ipm_matrix, image_points, world_points)
    display_reprojection(image, ipm_matrix, image_points, world_points, world=True)

    # Display calibration results
    print('ipm_matrix:', ipm_matrix.flatten().tolist())


def test_calibration(config_file, image_file, display=False):
    ipm = InversePerspectiveMapping(config_file)
    
    print(ipm.transform_points_to_px(ipm.in_points))
    print(ipm.transform_points_to_m(ipm.in_points))
    
    if display:
        win_name = 'Calibration Test'
        cv2.namedWindow(win_name)
        cv2.moveWindow(win_name, 100, 100)

        image = cv2.imread(image_file)
        output = ipm.transform_image(image)

        cv2.imshow(win_name, output)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    perform_calibration2('camera_info.yaml', 'ipm2.jpg')
    # test_calibration('camera_info.yaml', 'Calibration-04-00001--3.946949.jpg', display=True)
