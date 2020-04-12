# Dependencies
import json
import numpy as np
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt
import cv2
import glob
from operator import itemgetter


def json_to_numpy_mask(shapes, width, height):
    """
    Converts JSON labels with pixel classifications into NumPy arrays
    """
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
    """
    Organizes a collection of images or JSON files into a NumPy array
    """
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


if __name__ == "__main__":

    masks = create_dataset("/content/drive/My Drive/RoboJackets/RJ_Data/*.json", "json")
    images = create_dataset(
        "/content/drive/My Drive/RoboJackets/RJ_Data/*.png", "images"
    )

    # Reshape into NN-compatible format
    masks = np.reshape(masks, (741, 480, 640, 1))

    # Save NumPy arrays as .npy files

    np.save("images.npy", images)
    np.save("masks.npy", masks)
