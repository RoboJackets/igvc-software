import cv2
import numpy as np
import pylab as pl
import random as rand
from math import sqrt, atan, degrees

ground_plane_size = 90
pot_hole_radius = .15
image_size = 4000

def genIntermediateCourse():
    blank_image = np.zeros((image_size,image_size,3))
    b_channel, g_channel, r_channel = cv2.split(blank_image)
    alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255
    
    outerRad = rand.randint(10, 15);
    innerRad = outerRad - 2;
    centerRad = innerRad - 1;

    create_circle(convert_distance_to_pixel(39.2), int(image_size / 2), innerRad, outerRad, 225, 220, alpha_channel, 0)
    create_circle(convert_distance_to_pixel(39.2), int(image_size / 2), 0, centerRad, 0, 360, alpha_channel, 0)
    create_circle(convert_distance_to_pixel(39.2), int(image_size / 2), centerRad, innerRad, 250, 290, alpha_channel, 0)

    img_RGBA = cv2.merge((b_channel, g_channel, r_channel, alpha_channel))
    cv2.imwrite("blended_texture.png", img_RGBA)



def genBasicCourse(x):
    for each in range(x):
        blank_image = np.zeros((image_size,image_size,3))
        b_channel, g_channel, r_channel = cv2.split(blank_image)
        alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255
        
        width = rand.randint(1,3)
        pixWidth = convert_distance_to_pixel(width)
        length = rand.randint(20,86)
        create_line(int(image_size / 2), image_size, length, width, alpha_channel)
        
        img_RGBA = cv2.merge((b_channel, g_channel, r_channel, alpha_channel))
        numObst = rand.randint(2,6)
        centerList = []
        pixelRadius = convert_distance_to_pixel(.15)
        for eachObs in range(numObst):
            f = image_size / 2 + convert_distance_to_pixel(rand.uniform(pot_hole_radius, width - pot_hole_radius))
            centerX = int(f)
            f = image_size - convert_distance_to_pixel(rand.uniform(pot_hole_radius, length - pot_hole_radius))
            centerY = int(f)
            while ((centerX, centerY) in centerList):
                centerX = convert_distance_to_pixel(rand.uniform(1 + pot_hole_radius, 3 - pot_hole_radius))
                centerY = convert_distance_to_pixel(rand.uniform(20 + pot_hole_radius, 86 - pot_hole_radius))
            centerList.append((centerX,centerY))
            cv2.circle(img_RGBA, (centerX,centerY), pixelRadius, (255,255,255,255),-1)

        cv2.imwrite("basicCourse" + str(each) + ".png", img_RGBA)
    
    


def create_circle(centerX, centerY, radiusIn, radiusOut, orientationStart, orientationEnd, channel, brightness):
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

            if distance <= radiusOut and distance >= radiusIn:
                if theta >= orientationStart and theta <= orientationEnd:
                    channel[j, i] = brightness
                elif orientationStart > orientationEnd:
                    if theta > orientationStart or theta < orientationEnd:
                        channel[j, i] = brightness

    return channel

def convert_pixel_to_distance(pixel1X, pixel1Y, pixel2X, pixel2Y):
    xDist = (pixel1X - pixel2X) * (float(ground_plane_size) / image_size)
    yDist = (pixel1Y - pixel2Y) * (float(ground_plane_size) / image_size)
    return sqrt(xDist**2 + yDist**2)

def convert_distance_to_pixel(distance):
    return int(round(distance * (float(image_size) / ground_plane_size)))

def create_line(startX, startY, length, width, alpha_channel):
    for i in range(startY - convert_distance_to_pixel(length), startY):
        for j in range(startX, startX + convert_distance_to_pixel(width)):
            alpha_channel[i, j] = 0
    return alpha_channel


def main():
    genBasicCourse(6)
    genIntermediateCourse()
    
if __name__=='__main__':
    main()
