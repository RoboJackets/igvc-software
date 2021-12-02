import cv2
import numpy as np
import logging
import time
import os
import matplotlib.pyplot as plt
import glob

root_folder = os.path.dirname(os.path.abspath(__file__))

# Load the class labels in which our YOLO model was trained on
labelsPath = os.path.join(root_folder, "custom_data/custom.names")
LABELS = open(labelsPath).read().strip().split("\n")

# derive the paths to the YOLO weights and model configuration
weightsPath = os.path.join(root_folder, "Model_Weights/yolov3-custom_5000.weights")
configPath = os.path.join(root_folder, "custom_data/cfg/yolov3-custom.cfg")

# Loading the neural network framework Darknet (YOLO was created based on this framework)
net = cv2.dnn.readNet(weightsPath, configPath)

# display function to show image on Jupyter
def display_img(img,cmap=None):
    fig = plt.figure(figsize = (12,12))
    plt.axis('off')
    ax = fig.add_subplot(111)
    ax.imshow(img,cmap)
    plt.show()

# Create the function which predict the frame input
def predict(image):
    
    # initialize a list of colors to represent each possible class label
    np.random.seed(42)
    COLORS = np.random.randint(0, 255, size=(len(LABELS), 3), dtype="uint8")
    (H, W) = image.shape[:2]
    
    # determine only the "ouput" layers name which we need from YOLO
    ln = net.getLayerNames()
    ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    
    # construct a blob from the input image and then perform a forward pass of the YOLO object detector, 
    # giving us our bounding boxes and associated probabilities
    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    layerOutputs = net.forward(ln)
    
    boxes = []
    confidences = []
    classIDs = []
    threshold = 0.1
    
    # loop over each of the layer outputs
    for output in layerOutputs:
        # loop over each of the detections
        for detection in output:
            # extract the class ID and confidence (i.e., probability) of
            # the current object detection
            scores = detection[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]

            # filter out weak predictions by ensuring the detected
            # probability is greater than the minimum probability
            # confidence type=float, default=0.5
            if confidence > threshold:
                # scale the bounding box coordinates back relative to the
                # size of the image, keeping in mind that YOLO actually
                # returns the center (x, y)-coordinates of the bounding
                # box followed by the boxes' width and height
                box = detection[0:4] * np.array([W, H, W, H])
                (centerX, centerY, width, height) = box.astype("int")

                # use the center (x, y)-coordinates to derive the top and
                # and left corner of the bounding box
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))

                # update our list of bounding box coordinates, confidences,
                # and class IDs
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                classIDs.append(classID)

    # apply non-maxima suppression to suppress weak, overlapping bounding boxes
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, threshold, 0.1)

    # ensure at least one detection exists
    if len(idxs) > 0:
        # loop over the indexes we are keeping
        for i in idxs.flatten():
            # extract the bounding box coordinates
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])

            # draw a bounding box rectangle and label on the image
            color = (0,0,0)
            cv2.rectangle(image, (x, y), (x + w, y + h), color, -1)
            text = "{} {: .3f}".format(LABELS[classIDs[i]], confidences[i])
            cv2.putText(image, text, (x +15, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                1, color, 1)
    return image

def detect_edges(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    cv2.imshow("hsv", hsv)
    cv2.waitKey(0)
    # Lchannel = hls[:,:,1]
    # lower_blue = np.array([60, 40, 40])
    # upper_blue = np.array([150, 255, 255])
    sensitivity = 70
    lower_white = np.array([0, 0, 255 - sensitivity])
    upper_white = np.array([255, sensitivity, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    cv2.imshow("mask", mask)
    cv2.waitKey(0)

    kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    cv2.imshow("morphology", mask)
    cv2.waitKey(0)

    # detect edges
    edges = cv2.Canny(mask, 200, 400)
    cv2.imshow("edges", edges)
    cv2.waitKey(0)

    return edges


def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold,
                                    np.array([]), minLineLength=1, maxLineGap=7)
    # print(line_segments)
    return line_segments


def create_arrays(frame, line_segments):
    height, width, _ = frame.shape
    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary  # right lane line segment should be on left 2/3 of the screen

    right_lane = []
    left_lane = []
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_lane.append((x1, y1))
                    left_lane.append((x2, y2))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_lane.append((x1, y1))
                    right_lane.append((x2, y2))
    if len(left_lane) < 10:
        left_lane = []
    if len(right_lane) < 10:
        right_lane = []
    print(left_lane)
    print(right_lane)
    return left_lane, right_lane


def is_outlier(points, thresh=1):
    """
    Returns a boolean array with True if points are outliers and False
    otherwise.
    Parameters:
    -----------
        points : An numobservations by numdimensions array of observations
        thresh : The modified z-score to use as a threshold. Observations with
            a modified z-score (based on the median absolute deviation) greater
            than this value will be classified as outliers.
    Returns:
    --------
        mask : A numobservations-length boolean array.
    References:
    ----------
        Boris Iglewicz and David Hoaglin (1993), "Volume 16: How to Detect and
        Handle Outliers", The ASQC Basic References in Quality Control:
        Statistical Techniques, Edward F. Mykytka, Ph.D., Editor.
    """
    if len(np.shape(points)) == 3.5:
        points = points[:,None]
    median = np.median(points, axis=0)
    diff = np.sum((points - median)**2, axis=-1)
    diff = np.sqrt(diff)
    med_abs_deviation = np.median(diff)

    modified_z_score = 0.6745 * diff / med_abs_deviation
    print(modified_z_score > thresh)
    return modified_z_score > thresh

def remove_outliers(left_lane, right_lane):
    new_left_lane = []
    new_right_lane = []
    left_lane_outliers = is_outlier(left_lane).tolist()
    right_lane_outliers = is_outlier(right_lane).tolist()
    for i in left_lane_outliers:
        if not left_lane_outliers[i]:
            new_left_lane.append(left_lane[i])
    for i in right_lane_outliers:
        if not right_lane_outliers[i]:
            new_right_lane.append(right_lane[i])
    print(new_right_lane)
    print(new_left_lane)
    return new_left_lane, new_right_lane

def remove_toofar(lane):
    lane_list = []
    pt_init = lane[0]
    for pt in lane[1:]:
        dist = np.linalg.norm(pt-pt_init)
        if dist < 5:
            lane_list.append(pt_init)
            lane_list.append(pt)
        pt_init = pt
    return np.asarray(lane_list)


def detect_lane(frame):
    edges = detect_edges(frame)
    # cropped_edges = region_of_interest(edges)
    line_segments = detect_line_segments(edges)
    # lane_lines = average_slope_intercept(frame, line_segments)
    left_lane, right_lane = create_arrays(frame, line_segments)

    sorted_left = sorted(left_lane, key=lambda x: x[1])
    sorted_right = sorted(right_lane, key=lambda x: x[1])

    left_lane = np.asarray(sorted_left, dtype=np.int32)
    right_lane = np.asarray(sorted_right, dtype=np.int32)

    if (left_lane.shape[0] < 5):
        left_lane = np.asarray([])
    if (right_lane.shape[0] < 5):
        right_lane = np.asarray([])   

    if (left_lane.shape[0] > 1):
        left_lane = remove_toofar(left_lane)
    if (right_lane.shape[0] > 1):
        right_lane = remove_toofar(right_lane)

    # return lane_lines
    return left_lane, right_lane


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def draw_curves(frame, left_lane, right_lane, line_color=(0, 255, 0), line_width=2):
    # new_left_lane, new_right_lane = remove_outliers(left_lane, right_lane)
    print(left_lane)
    print(right_lane)
    line_image = np.zeros_like(frame)
    cv2.polylines(line_image, [left_lane], False, (0, 255, 255))
    cv2.polylines(line_image, [right_lane], False, (255, 255, 255))
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

# Execute prediction on a single image
photo_directory = os.path.join(root_folder, "Test_Images")
for filename in glob.glob(photo_directory + "/*.png"):
    print(filename)
    frame = cv2.imread(filename)
    frame = predict(frame)
    left_lane, right_lane = detect_lane(frame)
    lane_lines_image = draw_curves(frame, left_lane, right_lane)
    # lane_lines = detect_lane(frame)
    # lane_lines_image = display_lines(frame, lane_lines)
    cv2.imshow("lane lines", lane_lines_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

"""
# frame = cv2.imread('/home/vivek/Desktop/IGVC/IGVC_DATA(COMP)/JESSII_COMPO_RUN_3_2019-06-08-17-12-31/center_frame000236.png')
# frame = cv2.imread('/home/vivek/Desktop/IGVC/IGVC_DATA(COMP)/JESSII_COMPO_RUN_3_2019-06-08-17-12-31/center_frame000131.png')
frame = cv2.imread('/home/vivek/Desktop/IGVC/IGVC_DATA(COMP)/JESSII_COMPO_RUN_3_2019-06-08-17-12-31/center_frame001301.png')
/home/vivek/TrainYourOwnYOLO/Data/Source_Images/Test_Images/center_frame001292.png

frame = predict(frame)

left_lane, right_lane = detect_lane(frame)
lane_lines_image = draw_curves(frame, left_lane, right_lane)
# lane_lines = detect_lane(frame)
# lane_lines_image = display_lines(frame, lane_lines)

cv2.imshow("lane lines", lane_lines_image)
cv2.waitKey(0)

# line_segment = detect_line_segments(cropped_edges)
# lane_lines = average_slope_intercept(frame, line_segment)
"""