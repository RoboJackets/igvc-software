import cv2
import numpy as np
import pylab as pl
import random as rand
from math import sqrt, atan, degrees, sin, cos

#size of field, meters
ground_plane_size = 90

#size of image, pixels
image_size = 4000

#basic course options
basic_lower_length = 20
basic_upper_length = 86

basic_lower_width = 3
basic_upper_width = 6

#intermediate course options
intermediate_lower_radius = 10
intermediate_upper_radius = 15

#pothole info
pot_hole_radius = .30
pot_hole_separation = 1.8

#clearance
five_feet = 67

#obstacle bounds, intermediate
lower_boundI = 6
upper_boundI = 10

#obstacle bounds, basic
lower_boundB = 2
upper_boundB = 6

def genIntermediateCourse(x):
    for each in range(x):
        blank_image = np.zeros((image_size,image_size,3))
        b_channel, g_channel, r_channel = cv2.split(blank_image)
        alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255
        
        outerRad = rand.randint(intermediate_lower_radius, intermediate_upper_radius + 1);
        innerRad = outerRad - 3;
        centerRad = innerRad - 1;

        slitAngle = rand.randint(5, 300)
        openEntrance = slitAngle + rand.randint(5, 30)
        create_circle(int(image_size/2), int(image_size / 2), innerRad, outerRad, slitAngle, slitAngle - 5, alpha_channel, 0)
        create_circle(int(image_size/2), int(image_size / 2), 0, centerRad, 0, 360, alpha_channel, 0)
        create_circle(int(image_size/2), int(image_size / 2), centerRad, innerRad, openEntrance, openEntrance + 40, alpha_channel, 0)

        img_RGBA = cv2.merge((b_channel, g_channel, r_channel, alpha_channel))
        numObst = rand.randint(lower_boundI, upper_boundI + 1)
        centerList = []

        pixelRadius = convert_distance_to_pixel(.15)
        centerRadPix = convert_distance_to_pixel(centerRad)
        innerRadPix = convert_distance_to_pixel(innerRad)
        outerRadPix = convert_distance_to_pixel(outerRad)
        
        for eachObs in range(numObst):
            pointPixDist = rand.uniform(0, centerRadPix - pixelRadius - five_feet)
            centerX = int(rand.uniform(-pointPixDist, pointPixDist))
            if(rand.randint(0,10) / 10.0 > 0.5):
                centerY = int(sqrt(pointPixDist**2 - centerX**2))
            else:
                centerY = int(-sqrt(pointPixDist**2 - centerX**2))
            done = False
            centerList.append((centerX, centerY))
            cv2.circle(img_RGBA, (int(image_size / 2) + centerX,int(image_size / 2) + centerY), int(pixelRadius), (255,255,255,255),-1)
        centerList = []
        for eachObs in range(numObst):
            pointPixDist = rand.uniform(innerRadPix + pixelRadius + five_feet, outerRadPix - pixelRadius)
            centerX = int(rand.uniform(-pointPixDist, pointPixDist))
            if(rand.randint(0,10) / 10.0 > 0.5):
                centerY = int(sqrt(pointPixDist**2 - centerX**2))
            else:
                centerY = int(-sqrt(pointPixDist**2 - centerX**2))
            centerList.append((int(centerX), int(centerY)))
            cv2.circle(img_RGBA, ((int(image_size / 2) + centerX),int((image_size / 2) + centerY)), int(pixelRadius), (255,255,255,255),-1)
        cv2.imwrite("blended_texture" + str(each) + ".png", img_RGBA)



def genBasicCourse(x):
    for each in range(x):
        blank_image = np.zeros((image_size,image_size,3))
        b_channel, g_channel, r_channel = cv2.split(blank_image)
        alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255
        
        width = rand.randint(basic_lower_width, basic_upper_width)
        pixWidth = convert_distance_to_pixel(width)

        length = rand.randint(basic_lower_length, basic_upper_length)
        create_line(int(image_size / 2), image_size, length, width, alpha_channel)
        
        img_RGBA = cv2.merge((b_channel, g_channel, r_channel, alpha_channel))
        numObst = rand.randint(lower_boundB ,upper_boundB + 1)
        centerList = []
        pixelRadius = convert_distance_to_pixel(.15)
        for eachObs in range(numObst):
            f = image_size / 2 + convert_distance_to_pixel(rand.uniform(pot_hole_radius, width - pot_hole_radius))
            centerX = int(f)
            f = image_size - convert_distance_to_pixel(rand.uniform(pot_hole_radius, length - pot_hole_radius))
            centerY = int(f)
            newHole = (centerX,centerY)
            done = False
            i = 0
            while(not done):
                if (i != len(centerList)):
                    eachPot = centerList[i]
                    if (convert_pixel_to_distance(newHole[0], newHole[1], eachPot[0], eachPot[1]) <= pot_hole_separation):
                        centerX = int(centerX + pot_hole_separation)
                        centerY = int(convert_distance_to_pixel(rand.uniform(20 + pot_hole_radius, 86 - pot_hole_radius)))
                        newHole = (centerX,centerY)
                        i = 0
                    else:
                        i += 1
                else:
                    done = True
            centerList.append(newHole)
            cv2.circle(img_RGBA, (centerX,centerY), pixelRadius, (255,255,255,255),-1)

        cv2.imwrite("basicCourse" + str(each) + ".png", img_RGBA)
    
    


def create_circle(centerX, centerY, radiusIn, radiusOut, orientationStart, orientationEnd, channel, brightness):
    for i in range(centerX - convert_distance_to_pixel(radiusOut), centerX + convert_distance_to_pixel(radiusOut)):
        for j in range(centerY - convert_distance_to_pixel(radiusOut), centerY + convert_distance_to_pixel(radiusOut)):
            distance = convert_pixel_to_distance(centerX, centerY, i, j)
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

#TODO: Intermediate Course does NOT spawn obstacles near the interior of the outer circle.
#   The basic course does NOT spawn obstacles in close poximity to one another.
#   These limitations are a result of trying to keep 5 ft of clearance, so that the robot can navigate the course.
#   A soltuion to this would be welcome!
def main():
    genBasicCourse(10)
    genIntermediateCourse(10)
    
if __name__=='__main__':
    main()
