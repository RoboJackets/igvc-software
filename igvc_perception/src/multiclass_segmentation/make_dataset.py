# Dependencies
import json
import numpy as np
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt
import cv2
import glob
from operator import itemgetter
import sys
import argparse

num_images = 741


def json_to_numpy_mask(shapes, width, height):
    # Converts JSON labels with pixel classifications into NumPy arrays
    img = Image.new("L", (width, height), 0)
    for shape in shapes:
        if shape["label"] == "barrel":
            barrel_lst = [tuple(i) for i in shape["points"]]
            ImageDraw.Draw(img).polygon(barrel_lst, outline=1, fill=1)
        if shape["label"] == "line":
            line_lst = [tuple(i) for i in shape["points"]]
            ImageDraw.Draw(img).polygon(line_lst, outline=2, fill=2)
    mask = np.array(img)
    return mask


def create_dataset(path_to_folder, file_type):
    # Organizes a collection of images or JSON files into a NumPy array
    all_data = []
    files = glob.glob(path_to_folder)
    for temp_name in files:
        print("loading " + str(temp_name))
        if file_type == "images":
            temp_data = cv2.imread(temp_name)
        elif file_type == "json":
            f = open(temp_name, encoding="utf-8")
            loaded = json.load(f)
            temp_data = json_to_numpy_mask(loaded["shapes"], 640, 480)
        elif file_type == "numpy":
            print(file_type)
        else:
            print(file_type)
        temp = [temp_name, temp_data]
        all_data.append(temp)
    all_data = sorted(all_data, key=itemgetter(0))
    all_data = [data.pop(1) for data in all_data]
    all_data = np.asarray(all_data)
    return all_data


# Establish arguments as file paths
ap = argparse.ArgumentParser()

ap.add_argument("-a", "--images", required=True, help="path to folder with images")
ap.add_argument("-b", "--masks", required=True, help="path to folder with masks")
args = vars(ap.parse_args())

images_path = args["images"]
masks_path = args["masks"]

images = create_dataset(images_path, "images")
masks = create_dataset(masks_path, "json")

images_shape = np.shape(images)
masks_shape = np.shape(masks)

if images_shape[0] != masks_shape[0]:
    print("There is not the same number of images and masks. Examine your folders.")

masks = np.reshape(masks, (num_images, 480, 640, 1))

# Save NumPy arrays as .npy files
np.save("images.npy", images)
np.save("masks.npy", masks)
