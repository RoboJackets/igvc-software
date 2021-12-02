#!/usr/bin/env python
import os
import cv2 as cv
import numpy as np


"""
class OpenCVModel(object):
    
    OpenCV morphological transform, canny edge detector, and hough line transform for line detection.
    
    def __init__(self):
"""

def canny(img_src):
    gray = cv.cvtColor(img_src, cv.COLOR_BGR2GRAY); 
    gray = cv.GaussianBlur(gray, (10, 5), cv.BORDER_DEFAULT)
    edges = cv.Canny(gray, 50, 150, apertureSize=3)
    return edges

def roi(canny):
    poly = np.array([[(200, canny.shape[0]), (1100, canny.shape[0]), (550, 250)]])
    mask = np.zeros_like(canny)
    cv.fillPoly(mask, poly, 255)
    roi = cv.bitwise_and(canny, mask)
    return roi

def get_lines(img_hsv, hough):
    img_out = np.zeros_like(img_hsv)

    if hough is not None:
        for line in hough:
            x1, y1, x2, y2 = line.reshape(4)
            cv.line(img_out, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return img_out        


src_folder = os.path.join(os.getcwd(), "img_src")
out_folder = os.path.join(os.getcwd(), "img_out")


for filename in os.listdir(src_folder):
    img_src = cv.imread(os.path.join(src_folder, filename))
    img_hsv = cv.cvtColor(img_src, cv.COLOR_BGR2HSV)

    img = cv.medianBlur(img_hsv, 5)

    # Closing operation (erode then dilate) to remove noise
    img = cv.erode(img, None, iterations=1)
    img = cv.dilate(img, None, iterations=1)

    # Opening operation (dilate then erode) to fill in holes
    img = cv.dilate(img, None, iterations=30)
    img = cv.erode(img, None, iterations=30)
    
    edges = canny(img_src) 
    img_cropped = roi(edges)
    hough = cv.HoughLinesP(img_cropped, 2, np.pi/180, 100, np.array([]), minLineLength = 40, maxLineGap=5)

    img_out = get_lines(img_hsv, hough)

    cv.imwrite(os.path.join(out_folder , filename), edges)

    cv.waitKey(0)

print("done")