# OpenCV based Line Detection

*Issue #Number*
#564

**Author:**
- Vivek Mhatre

## The Problem

Currently line detection is done using a neural network. We would like to implement line detection with OpenCV as an alternative if the neural network does not work well. Additionally, OpenCV is a useful library to use as it can do useful tasks like thresholding and edge detection. 
The color of the line, shape of the line, camera orientation, and position of the image are some challenges that come to mind when trying to detect the line.
The end goal of this project is to have line detection implemented with OpenCV.

## Proposed Solution

Depending on the color of the lines I would use one or two masks. The algorithm I would use to detect lines would go as follows.
1. Use BGR to HSV transformation to make lines the same color
2. Filter out pixels using a mask to isolate line color. Possibly a white and yellow mask?
3. Process Gaussian Blur to reduce image noise and help smooth out edges in the image.
4. Use a morphological transformation to further remove noise in the image. Will most likely use opening, which just erosion followed by dilation.
5. Process edge detection using Canny
6. Use Hough Transform (HoughLinesP) to get lines
7. Draw lines on source image

To solve this problem, I will first implement my algorithm using python and
Steps to solve issue:
1. I will first implement my algorithm using Python.
2. Use the training images on the robojackets cloud to tune my parameters.
3. Then I will implement my algorithm using C++
4. Test the performance in past simulations using ros bag files.

Challenges:
How to handle curves in the track:
- Segment the image and handle in small parts.
How to avoid detecting barrels with white lines:
- Try to capture lines with a green background (grass) and reject lines with an orange background (barrels)

## Questions & Research

Not sure about the line color but I can find that out by looking at the training images.
How will I handle curves in the course?
How will I handle white lines in the barrel?
How will varying weather conditions affect line detection?
What topic will I need to subscribe to in order to get images from the ros bag files when testing?

## Overall Scope

### Affected Packages

I will need the OpenCV package and line_layer.cpp. I will be making changes to the igvc_navigation package.

### Schedule

Subtask 1 (April 19th): Finish implementing algorithm in python.

Subtask 2 (April 26th): Implement algorithm in C++.

Subtask 3 (April 29th): Test algorithm using ros bag files.

Code Review (May 1st): Everything should be done now.
