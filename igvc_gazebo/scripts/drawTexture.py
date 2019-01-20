#!/usr/bin/env python

import cv2
import numpy as np
from math import sqrt, atan, degrees

# tuple for the size of the field X, Y
ground_plane_size = 100

# pixel width and height
image_size = 4000
"""
  alters the array passed in that will be used to generate the alpha of the image
  centerX, centerY: the pixel coordinates that will be the center of the circle
  radiusIn: the inner radius of the arc
  radiusOut: the outer radiius of the arc
  orientationStart: the starting angle of the arc in degrees starting at 0 to the right and increasing counter clockwise
  orientationEnd: the ending angle of the arc in degrees
  alpha_channel: the array to be modified
"""
def create_circle(centerX, centerY, radiusIn, radiusOut, orientationStart, orientationEnd, alpha_channel):
    for i in range(centerX - convert_distance_to_pixel(radiusOut), centerX + convert_distance_to_pixel(radiusOut)):
        for j in range(centerY - convert_distance_to_pixel(radiusOut), centerY + convert_distance_to_pixel(radiusOut)):
            distance = convert_pixel_to_distance(centerX, centerY, i, j)
            # gets angle from our defined zero (right)
            theta = -1
            if i - centerX != 0:
                theta = degrees(atan((float(j) - centerY)/(i - centerX)))

            if i >= centerX and j <= centerY:
                theta += 360
            elif i <= centerX and j >= centerY:
                theta += 180
            elif i <= centerX and j <= centerY:
                theta += 180

            if distance <= radiusOut and distance >= radiusIn and theta >= orientationStart and theta <= orientationEnd:
                alpha_channel[j, i] = 0

    return alpha_channel

# converts the distance between the two pixels into meters
def convert_pixel_to_distance(pixel1X, pixel1Y, pixel2X, pixel2Y):
    xDist = (pixel1X - pixel2X) * (float(ground_plane_size) / image_size)
    yDist = (pixel1Y - pixel2Y) * (float(ground_plane_size) / image_size)
    return sqrt(xDist**2 + yDist**2)

# converts the distance (in meters) to pixel distance
def convert_distance_to_pixel(distance):
    return int(round(distance * (float(image_size) / ground_plane_size)))

"""
  draws a line at the given center with the given width and length
  startX, startY the starting point of the line
  width: the distance in meters of how wide the line should be
  length: the distance in meters of how long the line should be
  alpha_channel: the array to alter
"""
def create_line(startX, startY, width, length, alpha_channel):
    half_width = float(width) / 2
    for i in range(startY - convert_distance_to_pixel(half_width), startY + convert_distance_to_pixel(half_width)):
        for j in range(startX, startX + convert_distance_to_pixel(length)):
            alpha_channel[i, j] = 0
    return alpha_channel


def main():
    # creates a blank image and splits the color channels
    blank_image = np.zeros((image_size,image_size,3))
    b_channel, g_channel, r_channel = cv2.split(blank_image)
    alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255

    # changes the alpha values as necessary
    # 6, 5, 4, 3 60 meters straight
    create_line(convert_distance_to_pixel(20), convert_distance_to_pixel(20), 0.0762, 60, alpha_channel)
    create_line(convert_distance_to_pixel(20), convert_distance_to_pixel(26), 0.0762, 60, alpha_channel)
    create_line(convert_distance_to_pixel(20), convert_distance_to_pixel(31), 0.0762, 60, alpha_channel)
    create_line(convert_distance_to_pixel(20), convert_distance_to_pixel(35), 0.0762, 60, alpha_channel)
    create_line(convert_distance_to_pixel(20), convert_distance_to_pixel(38), 0.0762, 60, alpha_channel)

    # 6, 5, 4, 3 40 meters straight, 20 meters wiggly
    create_line(convert_distance_to_pixel(20), convert_distance_to_pixel(44), 0.0762, 40, alpha_channel)
    create_line(convert_distance_to_pixel(79), convert_distance_to_pixel(44), 0.0762, 1, alpha_channel)

    create_line(convert_distance_to_pixel(20), convert_distance_to_pixel(49), 0.0762, 40, alpha_channel)
    create_line(convert_distance_to_pixel(79), convert_distance_to_pixel(49), 0.0762, 1, alpha_channel)

    create_line(convert_distance_to_pixel(20), convert_distance_to_pixel(53), 0.0762, 40, alpha_channel)
    create_line(convert_distance_to_pixel(79), convert_distance_to_pixel(53), 0.0762, 1, alpha_channel)

    create_line(convert_distance_to_pixel(20), convert_distance_to_pixel(56), 0.0762, 40, alpha_channel)
    create_line(convert_distance_to_pixel(79), convert_distance_to_pixel(56), 0.0762, 1, alpha_channel)

    #6, 5, 4 20 meters straight, 40 meters wiggly
    create_line(convert_distance_to_pixel(20), convert_distance_to_pixel(62), 0.0762, 20, alpha_channel)
    create_line(convert_distance_to_pixel(79), convert_distance_to_pixel(62), 0.0762, 1, alpha_channel)

    create_line(convert_distance_to_pixel(20), convert_distance_to_pixel(67), 0.0762, 20, alpha_channel)
    create_line(convert_distance_to_pixel(79), convert_distance_to_pixel(67), 0.0762, 1, alpha_channel)

    create_line(convert_distance_to_pixel(20), convert_distance_to_pixel(71), 0.0762, 20, alpha_channel)
    create_line(convert_distance_to_pixel(79), convert_distance_to_pixel(71), 0.0762, 1, alpha_channel)




    # writes out the image
    img_RGBA = cv2.merge((b_channel, g_channel, r_channel, alpha_channel))
    cv2.imwrite("gen/blended_texture.png", img_RGBA)

if __name__ == '__main__':
    main()
