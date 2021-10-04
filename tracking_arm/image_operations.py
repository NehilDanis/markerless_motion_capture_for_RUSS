import os
import numpy as np
import cv2
import xmltodict
import torch
import torchvision
from d2l import torch as d2l

def change_file_names(folder_name):
    entries = os.listdir(folder_name)
    file_id = 1
    for entry in entries:
        os.rename(os.path.join(folder_name, entry), os.path.join(folder_name, "img" + str(file_id) + ".jpg"))
        file_id += 1


def read_point_from_xml(annotations_file):
    # Reading data from the xml file
    with open(annotations_file, 'r') as f:
        data = f.read()

    data_dict = xmltodict.parse(data)
    objects  = data_dict['annotation']['object']
    arm_w_tarso = None
    arm = None
    for object in objects:
        if object['name'] == 'arm with tarso':
            arm_w_tarso = object['polygon']['pt']
        elif object['name'] == 'arm':
            arm = object['polygon']['pt']
        else:
            print('incorrect part')

    r_arm_w_tarso = []
    r_arm = []
    for pt in arm_w_tarso:
        r_arm_w_tarso.append([int(pt['x']), int(pt['y'])])
    for pt in arm:
        r_arm.append([int(pt['x']), int(pt['y'])])

    return np.array(r_arm_w_tarso), np.array(r_arm)

def apply(img, mask, aug, num_rows=1, num_cols=1, scale=1.5):
    Y = [aug(img) for _ in range(num_rows * num_cols)]
    d2l.show_images(Y, num_rows, num_cols, scale=scale)

def augment_data(img, mask, idx, folder_name):
    data_dir = os.path.join(folder_name, "images")
    mask_dir = os.path.join(folder_name, "masks")
    if not os.path.isdir(data_dir):
        os.mkdir(data_dir)

    if not os.path.isdir(mask_dir):
        os.mkdir(mask_dir)

    # transform the numpy array to PIL image to be able to apply the torchvision trasnforms
    img_t = torchvision.transforms.ToPILImage()(img)
    mask_t = torchvision.transforms.ToPILImage()(mask)

    '''
    Here apply augmentation
    
    1 ) add normal image and mask to the dataset
    2 ) flip normal image and mask and add it to the dataset --> either horizontally or vertically
    3 ) change the brigtness of the normal image and add it to the dataset
    4 ) change the hue of the normal image and add it to the dataset
    5 ) random flip + change of brightness
    6 ) random flip + change of hue
    7 ) random flip + change of brightness + change of hue
    8 ) normal image + change of brightness + change of hue 
    9 ) normal image + change of saturation 
    10 ) normal image + change of contrast
    11 ) random flip + change of contrast 
    12 ) random flip + change of saturation 
    
    In the end we will have 12 images+masks created from 1 image+mask
    '''

    # 1 ) add normal image and mask to the dataset
    img_file = 'img' + str(idx) + ".jpg"
    mask_file = 'mask' + str(idx) + ".jpg"

    cv2.imwrite(os.path.join(data_dir, img_file), img)
    cv2.imwrite(os.path.join(mask_dir, mask_file), mask)
    idx += 1

    # 2 ) flip normal image and mask and add it to the dataset --> either horizontally or vertically
    img_file = 'img' + str(idx) + ".jpg"
    mask_file = 'mask' + str(idx) + ".jpg"

    transform = torchvision.transforms.RandomChoice([
        torchvision.transforms.RandomHorizontalFlip(p=1),
        torchvision.transforms.RandomVerticalFlip(p=1)
    ])

    cv2.imwrite(os.path.join(data_dir, img_file), np.array(transform(img_t)))
    cv2.imwrite(os.path.join(mask_dir, mask_file), np.array(transform(mask_t)))
    idx += 1

    # 3 ) change the brigtness of the normal image and add it to the dataset

    img_file = 'img' + str(idx) + ".jpg"
    mask_file = 'mask' + str(idx) + ".jpg"

    color_jitter = torchvision.transforms.ColorJitter(brightness=0.5, contrast=0, saturation=0, hue=0)
    transform = torchvision.transforms.ColorJitter.get_params(
        color_jitter.brightness, color_jitter.contrast, color_jitter.saturation,
        color_jitter.hue)

    cv2.imwrite(os.path.join(data_dir, img_file), np.array(transform(img_t)))
    cv2.imwrite(os.path.join(mask_dir, mask_file), mask)

    idx += 1

    # 4 ) change the hue of the normal image and add it to the dataset

    img_file = 'img' + str(idx) + ".jpg"
    mask_file = 'mask' + str(idx) + ".jpg"

    color_jitter = torchvision.transforms.ColorJitter(brightness=0, contrast=0, saturation=0, hue=0.5)
    transform = torchvision.transforms.ColorJitter.get_params(
        color_jitter.brightness, color_jitter.contrast, color_jitter.saturation,
        color_jitter.hue)

    cv2.imwrite(os.path.join(data_dir, img_file), np.array(transform(img_t)))
    cv2.imwrite(os.path.join(mask_dir, mask_file), mask)

    idx += 1

    # 5 ) random flip + change of brightness

    img_file = 'img' + str(idx) + ".jpg"
    mask_file = 'mask' + str(idx) + ".jpg"

    color_jitter = torchvision.transforms.ColorJitter(brightness=0.5, contrast=0, saturation=0, hue=0)
    transform_c = torchvision.transforms.ColorJitter.get_params(
        color_jitter.brightness, color_jitter.contrast, color_jitter.saturation,
        color_jitter.hue)

    transform = torchvision.transforms.RandomChoice([
        torchvision.transforms.RandomHorizontalFlip(p=1),
        torchvision.transforms.RandomVerticalFlip(p=1)
    ])

    cv2.imwrite(os.path.join(data_dir, img_file), np.array(transform_c(transform(img_t))))
    cv2.imwrite(os.path.join(mask_dir, mask_file), np.array(transform(mask_t)))

    idx += 1

    # 6 ) random flip + change of hue

    img_file = 'img' + str(idx) + ".jpg"
    mask_file = 'mask' + str(idx) + ".jpg"

    color_jitter = torchvision.transforms.ColorJitter(brightness=0, contrast=0, saturation=0, hue=0.5)
    transform_c = torchvision.transforms.ColorJitter.get_params(
        color_jitter.brightness, color_jitter.contrast, color_jitter.saturation,
        color_jitter.hue)

    transform = torchvision.transforms.RandomChoice([
        torchvision.transforms.RandomHorizontalFlip(p=1),
        torchvision.transforms.RandomVerticalFlip(p=1)
    ])

    cv2.imwrite(os.path.join(data_dir, img_file), np.array(transform_c(transform(img_t))))
    cv2.imwrite(os.path.join(mask_dir, mask_file), np.array(transform(mask_t)))

    idx += 1

    # 7 ) random flip + change of brightness + change of hue

    img_file = 'img' + str(idx) + ".jpg"
    mask_file = 'mask' + str(idx) + ".jpg"

    color_jitter = torchvision.transforms.ColorJitter(brightness=0.5, contrast=0, saturation=0, hue=0.5)
    transform_c = torchvision.transforms.ColorJitter.get_params(
        color_jitter.brightness, color_jitter.contrast, color_jitter.saturation,
        color_jitter.hue)

    transform = torchvision.transforms.RandomChoice([
        torchvision.transforms.RandomHorizontalFlip(p=1),
        torchvision.transforms.RandomVerticalFlip(p=1)
    ])

    cv2.imwrite(os.path.join(data_dir, img_file), np.array(transform_c(transform(img_t))))
    cv2.imwrite(os.path.join(mask_dir, mask_file), np.array(transform(mask_t)))

    idx += 1

    # 8 ) normal image + change of brightness + change of hue

    img_file = 'img' + str(idx) + ".jpg"
    mask_file = 'mask' + str(idx) + ".jpg"

    color_jitter = torchvision.transforms.ColorJitter(brightness=0, contrast=0, saturation=0, hue=0.5)
    transform = torchvision.transforms.ColorJitter.get_params(
        color_jitter.brightness, color_jitter.contrast, color_jitter.saturation,
        color_jitter.hue)

    cv2.imwrite(os.path.join(data_dir, img_file), np.array(transform(img_t)))
    cv2.imwrite(os.path.join(mask_dir, mask_file), mask)

    idx += 1

    # 9 ) normal image + change of saturation

    img_file = 'img' + str(idx) + ".jpg"
    mask_file = 'mask' + str(idx) + ".jpg"

    color_jitter = torchvision.transforms.ColorJitter(brightness=0, contrast=0, saturation=0.5, hue=0)
    transform = torchvision.transforms.ColorJitter.get_params(
        color_jitter.brightness, color_jitter.contrast, color_jitter.saturation,
        color_jitter.hue)

    cv2.imwrite(os.path.join(data_dir, img_file), np.array(transform(img_t)))
    cv2.imwrite(os.path.join(mask_dir, mask_file), mask)

    idx += 1

    # 10 ) normal image + change of contrast

    img_file = 'img' + str(idx) + ".jpg"
    mask_file = 'mask' + str(idx) + ".jpg"

    color_jitter = torchvision.transforms.ColorJitter(brightness=0, contrast=0.5, saturation=0, hue=0)
    transform = torchvision.transforms.ColorJitter.get_params(
        color_jitter.brightness, color_jitter.contrast, color_jitter.saturation,
        color_jitter.hue)

    cv2.imwrite(os.path.join(data_dir, img_file), np.array(transform(img_t)))
    cv2.imwrite(os.path.join(mask_dir, mask_file), mask)

    idx += 1

    # 11 ) random flip + change of contrast

    img_file = 'img' + str(idx) + ".jpg"
    mask_file = 'mask' + str(idx) + ".jpg"

    color_jitter = torchvision.transforms.ColorJitter(brightness=0, contrast=0.5, saturation=0, hue=0)
    transform_c = torchvision.transforms.ColorJitter.get_params(
        color_jitter.brightness, color_jitter.contrast, color_jitter.saturation,
        color_jitter.hue)

    transform = torchvision.transforms.RandomChoice([
        torchvision.transforms.RandomHorizontalFlip(p=1),
        torchvision.transforms.RandomVerticalFlip(p=1)
    ])

    cv2.imwrite(os.path.join(data_dir, img_file), np.array(transform_c(transform(img_t))))
    cv2.imwrite(os.path.join(mask_dir, mask_file), np.array(transform(mask_t)))

    idx += 1

    # 12 ) random flip + change of saturation

    img_file = 'img' + str(idx) + ".jpg"
    mask_file = 'mask' + str(idx) + ".jpg"

    color_jitter = torchvision.transforms.ColorJitter(brightness=0, contrast=0, saturation=0.5, hue=0)
    transform_c = torchvision.transforms.ColorJitter.get_params(
        color_jitter.brightness, color_jitter.contrast, color_jitter.saturation,
        color_jitter.hue)

    transform = torchvision.transforms.RandomChoice([
        torchvision.transforms.RandomHorizontalFlip(p=1),
        torchvision.transforms.RandomVerticalFlip(p=1)
    ])

    cv2.imwrite(os.path.join(data_dir, img_file), np.array(transform_c(transform(img_t))))
    cv2.imwrite(os.path.join(mask_dir, mask_file), np.array(transform(mask_t)))

    idx += 1

    return idx


def create_mask(im_folder_name, anotations_folder, mask_folder):
    entries = os.listdir(anotations_folder)
    file_id = 1
    idx_arm = 1
    idx_arm_w_tarso = 1
    arm_folder = ""
    for an_entry in entries:
        if an_entry.endswith('.xml'):
            img = np.asarray(cv2.imread(os.path.join(im_folder_name, an_entry[:-4] + ".jpg")))
            points_arm_w_tarso, points_arm = read_point_from_xml(os.path.join(anotations_folder, an_entry))

            file_name_arm  = os.path.join(mask_folder, an_entry[:-4] + "_arm.jpg")
            file_name_arm_w_tarso = os.path.join(mask_folder, an_entry[:-4] + "_arm_w_tarso.jpg")

            mask_arm = np.zeros(img.shape[:2], np.uint8)
            cv2.drawContours(mask_arm, [points_arm], -1, (255, 255, 255), -1, cv2.LINE_AA)

            ## (3) do bit-op
            dst_arm = cv2.bitwise_and(img, img, mask=mask_arm)
            cv2.fillPoly(dst_arm, pts=[points_arm], color=(255, 255, 255))
            cv2.imwrite(file_name_arm, dst_arm)

            mask_arm_w_tarso = np.zeros(img.shape[:2], np.uint8)
            cv2.drawContours(mask_arm_w_tarso, [points_arm_w_tarso], -1, (255, 255, 255), -1, cv2.LINE_AA)
            ## (3) do bit-op
            dst_arm_w_tarso = cv2.bitwise_and(img, img, mask=mask_arm_w_tarso)
            cv2.fillPoly(dst_arm_w_tarso, pts=[points_arm_w_tarso], color=(255, 255, 255))
            cv2.imwrite(file_name_arm_w_tarso, dst_arm_w_tarso)

            if file_id % 5 == 0:
                # validation set
                idx_arm = augment_data(img, dst_arm, idx_arm, "/home/nehil/new_arm_data_folder/Test")
                idx_arm_w_tarso = augment_data(img, dst_arm_w_tarso, idx_arm_w_tarso,
                                               "/home/nehil/new_arm_w_tarso_data_folder/Test")
            else:
                #train set
                idx_arm = augment_data(img, dst_arm, idx_arm, "/home/nehil/new_arm_data_folder/Train")
                idx_arm_w_tarso = augment_data(img, dst_arm_w_tarso, idx_arm_w_tarso, "/home/nehil/new_arm_w_tarso_data_folder/Train")
            file_id += 1


if __name__ == "__main__":
    # change_file_names("/var/www/html/LabelMeAnnotationTool/Images/example_folder")
    create_mask("/home/nehil/images_arm", "/home/nehil/annotations", "/var/www/html/LabelMeAnnotationTool/Masks/result_masks")