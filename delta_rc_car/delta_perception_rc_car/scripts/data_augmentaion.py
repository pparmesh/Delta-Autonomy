# first run command
# sudo pip install albumentations
import sys, os
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

from urllib.request import urlopen
import numpy as np
import cv2
from matplotlib import pyplot as plt

from albumentations import (
    BboxParams,
    HorizontalFlip,
    VerticalFlip,
    Resize,
    CenterCrop,
    RandomCrop,
    Crop,
    Compose
)

BOX_COLOR = (255, 0, 0)
TEXT_COLOR = (255, 255, 255)


def visualize_bbox(img, bbox, class_id, class_idx_to_name, color=BOX_COLOR, thickness=2):
    x_min, y_min, w, h = bbox
    x_min, x_max, y_min, y_max = int(x_min), int(x_min + w), int(y_min), int(y_min + h)
    cv2.rectangle(img, (x_min, y_min), (x_max, y_max), color=color, thickness=thickness)
    class_name = class_idx_to_name[class_id]
    ((text_width, text_height), _) = cv2.getTextSize(class_name, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)    
    cv2.rectangle(img, (x_min, y_min - int(1.3 * text_height)), (x_min + text_width, y_min), BOX_COLOR, -1)
    cv2.putText(img, class_name, (x_min, y_min - int(0.3 * text_height)), cv2.FONT_HERSHEY_SIMPLEX, 0.35,TEXT_COLOR, lineType=cv2.LINE_AA)
    return img


def visualize(annotations, category_id_to_name):
    img = annotations['image'].copy()
    for idx, bbox in enumerate(annotations['bboxes']):
        img = visualize_bbox(img, bbox, annotations['category_id'][idx], category_id_to_name)
    plt.figure(figsize=(12, 12))
    plt.imshow(img)

def get_aug(aug, min_area=0., min_visibility=0.):
    return Compose(aug, bbox_params=BboxParams(format='coco', min_area=min_area, 
                                               min_visibility=min_visibility, label_fields=['category_id']))

def save_augmented_img(annotations, category_id_to_name, orig_file, aug_type, directory):
    new_img = annotations['image'].copy()
    new_name = aug_type + orig_file + ".jpg"
    cv2.imwrite(directory + new_name, new_img)
    f= open(directory + new_name[:-4] + ".txt","w+")
    for idx, bbox in enumerate(annotations['bboxes']):
        label = annotations['category_id'][idx]
        f.write(str(label) + " "+ str((bbox[0] + bbox[2]/2)/new_img.shape[1])+ 
            " " + str((bbox[1] + bbox[3]/2)/new_img.shape[0])+ " "+ 
            str(bbox[2]/new_img.shape[1])+ " " + str(bbox[3]/new_img.shape[0]))
    f.close()

def roi_crop(img, size=[540, 720]):
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
            bbox_list = []
            category_list = []
            annotation_file = os.path.join(folder_name,filename)[:-3] + "txt"
            complete_text = open(annotation_file, "r")
            for line in complete_text:
                category_list.append(int(line.split()[0]))
                temp = [float(line.split()[1])*image.shape[1] - float(line.split()[3])*image.shape[1]/2, 
                        float(line.split()[2])*image.shape[0] - float(line.split()[4])*image.shape[0]/2, 
                        float(line.split()[3])*image.shape[1], 
                        float(line.split()[4])*image.shape[0]]
                bbox_list.append(temp)

            bbox_list = np.asarray(bbox_list)
            annotations = {'image': image, 'bboxes': bbox_list, 'category_id': category_list}

            # original image
            visualize(annotations, category_id_to_name)
            # plt.show()

            # Vertical flip
            aug = get_aug([VerticalFlip(p=1)])
            augmented = aug(**annotations)
            # visualize(augmented, category_id_to_name)
            # plt.show()
            save_augmented_img(augmented, category_id_to_name, filename[:-4], "vertical_flip_", augmentation_folder_name)

            # Horizontal flip
            aug = get_aug([HorizontalFlip(p=1)])
            augmented = aug(**annotations)
            # visualize(augmented, category_id_to_name)
            # plt.show()
            save_augmented_img(augmented, category_id_to_name, filename[:-4], "horizontal_flip_", augmentation_folder_name)

            # resize to square
            aug = get_aug([Resize(p=1, height=256, width=256)])
            augmented = aug(**annotations)
            # visualize(augmented, category_id_to_name)
            # plt.show()
            save_augmented_img(augmented, category_id_to_name, filename[:-4], "resize_", augmentation_folder_name)

            # center crop
            aug = get_aug([CenterCrop(p=1, height=300, width=300)])
            augmented = aug(**annotations)
            # visualize(augmented, category_id_to_name)
            # plt.show()
            save_augmented_img(augmented, category_id_to_name, filename[:-4], "center_crop_flip_", augmentation_folder_name)
            sys.exit()