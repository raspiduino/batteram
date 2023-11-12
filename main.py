# This file is responsible for all other stuff
# Created by gvl610

import cv2
import time
import math
import numpy as np
import os
from time import sleep
from const import *
from control import Arm

# Opening camera
# IMPORTANT NOTE: You must open the Windows Camera app first, then close it, before running this script
# Note: On Windows, we need to use CAP_DSHOW, or there will be unexpected results
print("[Detect] Openning camera")
cap = cv2.VideoCapture(CAM_ID, cv2.CAP_DSHOW)
if not cap.isOpened():
    raise IOError("[Detect] Cannot open webcam")

# Set resolution for the camera. You can change this to match your camera
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)

# Get first 10 frames, since first frames when the camera just init are really bad
for i in range(10):
    ret, inputImage = cap.read()
    if not ret:
        print("[Detect] Frame capture error!")

# Load the model. You can change to other models
print("[Detect] Loading model")
net = cv2.dnn.readNet(MODEL_NAME)
print("[Detect] Model loaded. Running on CPU")

# Set backend
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

# Find last image index
img_list = os.listdir("imglog\\")
img_list.sort()
img_index = int(img_list[-1].split(".")[0])

print(f"[Image] Last image index: {img_index}")

'''
run_detect: feed the image into net then return the result
This is where the image actually get detected
'''
def run_detect(image):
    blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
    t0 = time.time()
    net.setInput(blob)
    preds = net.forward()
    t1 = time.time()
    print("[Detect] run_detect time=", t1 - t0)
    return preds

'''
parse_all: parse all object's detection result
'''
def parse_all(data):
    # Place holder lists
    class_ids = []
    confidences = []
    boxes = []

    for r in range(data.shape[0]):
        # Get a row (a detected object)
        row = data[r]
        confidence = row[4]

        if confidence >= SCORE_THRESHOLD:
            # Append class confidence score
            confidences.append(confidence)

            # Append class id with best class score
            class_ids.append(cv2.minMaxLoc(row[5:])[3][1])

            # Get bounding box
            x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item()
            left = int(x - w/2)
            top = int(y - h/2)
            width = int(w)
            height = int(h)
            boxes.append(np.array([left, top, width, height]))

    # Filter boxes
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD)
    
    new_class_ids = []
    new_confidences = []
    new_boxes = []

    for i in indexes:
        new_class_ids.append(class_ids[i])
        new_confidences.append(confidences[i])
        new_boxes.append(boxes[i])

    # Return
    return new_class_ids, new_confidences, new_boxes

'''
get_orientation: get object's orientation from countour
'''
def get_orientation(pts):
    ## [pca]
    # Construct a buffer used by the pca analysis
    sz = len(pts)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i,0] = pts[i,0,0]
        data_pts[i,1] = pts[i,0,1]

    # Perform PCA analysis
    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
    ## [pca]

    angle = math.atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians

    return angle

'''
process_angle: get angle and draw contour, orientation from object image
'''
def process_angle(src):
    # Convert image to grayscale
    gray = None
    while True:
        try:
            gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
            break
        except Exception as e:
            print(e)
            continue

    # Convert image to binary
    _, bw = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    ## [pre-process]

    ## [contours]
    # Find all the contours in the thresholded image
    contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    l_area = 0
    l_c = None

    for i, c in enumerate(contours):
        # Calculate the area of each contour
        area = cv2.contourArea(c)
        # Ignore contours that are too small or too large
        if not area < 1e2 and not 1e5 < area:
            if area > l_area:
                l_area = area
                l_c = c

    # Find the orientation of each shape
    angle = get_orientation(l_c)

    return (src, angle)

# Init control
arm = Arm()
arm.connect("ws://192.168.43.199/ws")

# Move to home position and start the conveyor belt
arm.pwm[0] = DROP_S1_PWM
arm.pwm[1] = PREPARE_PWM[0]
arm.pwm[2] = PREPARE_PWM[1]
arm.pwm[3] = 170
arm.pwm[4] = GRIP_OFF
arm.pwm[5] = BELT_FORWARD[0]
arm.pwm[6] = BELT_FORWARD[1]
arm.send()

# A simple grip cycle
# Take d (object's distance from root) and s4 (servo 4's PWM value) as input
def grip_cycle(d, s4):
    # Move from home to pickup position and stop the coveyor belt
    arm.pwm[0] = PICK_S1_PWM
    arm.pwm[3] = s4
    arm.pwm[5] = BELT_STOP[0]
    arm.pwm[6] = BELT_STOP[1]
    arm.send()
    sleep(0.5)

    # Lookup the closest PWM values for moving the hand to distance d, then move it
    if d < 16:
        d = 16
    elif d > 24:
        d = 24
    p = PWM_TABLE[d]
    arm.pwm[1] = p[0]
    arm.pwm[2] = p[1]
    arm.send()
    sleep(0.5)

    # Grip the object
    arm.pwm[4] = GRIP_ON
    arm.send()
    sleep(0.8)

    # Move back to prepare position
    arm.pwm[1] = PREPARE_PWM[0]
    arm.pwm[2] = PREPARE_PWM[1]
    arm.pwm[3] = 170
    arm.send()
    sleep(0.5)

    # Move to drop position
    arm.pwm[0] = DROP_S1_PWM
    arm.send()
    sleep(1)

    # Drop the object
    arm.pwm[1] = DROP_PWM[0]
    arm.pwm[2] = DROP_PWM[1]
    arm.pwm[4] = GRIP_OFF
    arm.send()
    sleep(0.5)

    # Return to home position, end cycle
    arm.pwm[1] = PREPARE_PWM[0]
    arm.pwm[2] = PREPARE_PWM[1]
    arm.send()

# Main loop
while True:
    # Start the conveyor belt
    arm.pwm[5] = BELT_FORWARD[0]
    arm.pwm[6] = BELT_FORWARD[1]
    arm.send()

    if arm.ws.recv() == "found":
        # Start counting time
        t0 = time.time()

        # Stop the conveyor belt
        arm.pwm[5] = BELT_STOP[0]
        arm.pwm[6] = BELT_STOP[1]
        arm.send()

        # Wait for the object to stop moving
        sleep(0.4)

        # Capture frame
        ret, inputImage = cap.read()
        ret, inputImage = cap.read()
        ret, inputImage = cap.read()
        if not ret:
            print("[Detect] Frame capture error!")

        # Get the middle 704x704 part of the image
        inputImage = inputImage[0:704, 288:992]

        # Write the image (for retraining?)
        img_index += 1
        print("Saving image imglog\\" + str(img_index) + ".png")
        cv2.imwrite("imglog\\" + str(img_index) + ".png", inputImage)

        # Run detection
        outs = run_detect(inputImage)

        # Parse result
        class_ids, confidences, boxes = parse_all(outs[0])

        # Return if no object detected
        if len(class_ids) < 1:
            print("[Detect] Nothing detected!")
            arm.pwm[5] = BELT_FORWARD[0]
            arm.pwm[6] = BELT_FORWARD[1] - 1
            arm.send()
            sleep(1)
            continue

        # Get the object closest to the left on the image, and not out of Y range
        target_obj_id = 0
        target_obj_x0 = 9999
        for i in range(len(boxes)):
            x0 = boxes[i][0].item()
            y0 = boxes[i][1].item()
            h  = boxes[i][3].item()
            y1 = y0 + h
            if x0 < target_obj_x0 and y0 > D2Y[16] and y1 < D2Y[27]:
                # If current object is closer to the left than the previous one
                # and not out of Y range
                target_obj_x0 = x0
                target_obj_id = i

        # After we selected the object, get its info
        class_id = class_ids[target_obj_id]
        confidence = confidences[target_obj_id]
        box = boxes[target_obj_id]

        # Get object's box location
        y0 = box[1].item()
        h  = box[3].item()
        y1 = y0 + h
        yc = y0 + h//2 # Center

        x0 = box[0].item()
        w  = box[2].item()
        x1 = x0 + h
        xc = x0 + w//2 # Center

        # Print information about the selected object
        print(f"[Object] Object selected: {CLASS_NAMES[class_id]} @ {x0},{y0},{xc},{yc} (x0,y0,xc,yc) conf={confidence}")

        # If the object is out of gripping range -> run the coveyor belt in reverse direction
        if (xc < PROX_START):
            print(f"[Object] Out of gripping range. Moving it backward")
            arm.pwm[5] = BELT_BACKWARD[0]
            arm.pwm[6] = BELT_BACKWARD[1]
            arm.send()
        elif (xc > PROX_END):
            print(f"[Object] Out of gripping range. Moving it forward")
            arm.pwm[5] = BELT_FORWARD[0]
            arm.pwm[6] = BELT_FORWARD[1]
            arm.send()
        
        # Calculate object's y distance from arm's root
        d = round((D2Y[16] - yc)*P2C) + 16

        print(f"[Object] Distance = {d}cm")

        # Calculate object's orientation angle
        obj = inputImage[y0:y1, x0:x1] # Get only the object's image
        outimg, o_angle = process_angle(obj)
        inputImage[y0:y1, x0:x1] = outimg # Replace the object's image with the one has orientation drawn
        #o_angle = math.degrees(o_angle) # Convert to degree

        print(f"[Object] Object orientation angle: {o_angle} rad")

        # Calculate PWM value for servo 4
        a4 = 90 + o_angle*180/math.pi
        s4 = None
        if a4 >= 145:
            s4 = 170 - abs(180 - a4)*DEG2PWM
        else:
            s4 = 170 + a4*DEG2PWM
        
        # Do a grip cycle
        print(f"[Control] Do grip cycle")
        grip_cycle(d, s4)
        print(f"[Control] Grip cycle done!")

        # Print the time needed for the whole cycle
        t1 = time.time()
        print("[Cycle] Full cycle time=", t1 - t0)
