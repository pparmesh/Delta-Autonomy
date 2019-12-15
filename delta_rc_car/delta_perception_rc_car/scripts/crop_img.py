# first run command
# sudo pip install albumentations
import sys, os
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

from urllib.request import urlopen
import numpy as np
import cv2
from matplotlib import pyplot as plt


def roi_crop(img, size=[480, 720]):
    h, w, c = img.shape
    del_side = (w-size[1])/2
    del_top = h- size[0]
    cropped_img = img[int(del_top):, int(del_side):int(-del_side), :]
    return cropped_img

if __name__ == "__main__":
    category_id_to_name = {0: 'cat', 1: 'dog'}
    folder_name = "/home/apoorv/Desktop/delta/rc_car/darknet/data/img/"
    augmentation_folder_name = "../augmentations/"

    for filename in os.listdir(folder_name):
        image = cv2.imread(os.path.join(folder_name,filename), cv2.COLOR_BGR2RGB)
        image = roi_crop(image)
        sys.exit()
        if image is not None:
            image = roi_crop(image)
            cv2.imwrite(os.path.join(folder_name,filename), image)