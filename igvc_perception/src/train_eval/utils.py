import numpy as np
from math import floor

def read_data_cfg(datacfg):
    options = dict()
    with open(datacfg, 'r') as fp:
        lines = fp.readlines()

    for line in lines:
        line = line.strip()
        if line == '':
            continue
        key,value = line.split('=')
        key = key.strip()
        value = value.strip()
        options[key] = value
    return options


def slice_image(img, width, height=None, step_size=1):
    """
    Function that takes an image and slices it into multiple smaller images
    return in a numpy array
    """
    if height is None:
        height = width  # make it a square

    img = np.array(img)
    nrows, ncols = img.shape[0], img.shape[1]
    output_size = floor((ncols - width)/step_size) \
        * floor((nrows - height)/step_size)
    res = np.zeros([height, width, output_size])

    corners = np.array([[1, 1], [1, height], [width, 1], [width, height]]) - 1

    def move_right(corners, step_size):
        corners[:, 0] + step_size
        return corners

    def move_down(corners, step_size):
        corners[:, 1] + step_size
        return corners

    def crop_image(corners, image):
        return image[corners[0, 1]:corners[3, 1], corners[0, 0]:corners[3, 0]]

    z = 0
    for row in range(0, floor((nrows - height)/step_size)):
        for col in floor((ncols - width)/step_size):
            res[:, :, z] = crop_image(corners, img)
            corners = move_right(corners, step_size)
            z += 1  # update out index
        corners = move_down(corners, step_size)

    return res
