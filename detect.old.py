# This file is responsible for detecting the object and return coordinates and orientation

import cv2
import time
import math
import numpy as np
from const import *

class Detect():
    def __init__(self, cam_id, model_name, class_names, cam_w, cam_h):
        self.cam_id = cam_id
        self.model_name = model_name
        self.class_names = class_names
        self.cam_w = cam_w
        self.cam_h = cam_h
        self.colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]

        # Opening camera
        # IMPORTANT NOTE: You must open the Windows Camera app first, then close it, before running this script
        # Note: On Windows, we need to use CAP_DSHOW, or there will be unexpected results
        print("[Detect] Openning camera")
        self.cap = cv2.VideoCapture(self.cam_id, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            raise IOError("[Detect] Cannot open webcam")
        
        # Set resolution for the camera. You can change this to match your camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_h)

        # Load the model. You can change to other models
        print("[Detect] Loading model")
        self.net = cv2.dnn.readNet(self.model_name)
        print("[Detect] Model loaded. Running on CPU")

        # Set backend
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    '''
    run_detect: feed the image into net then return the result
    This is where the image actually get detected
    '''
    def run_detect(self, image):
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
        t0 = time.time()
        self.net.setInput(blob)
        preds = self.net.forward()
        t1 = time.time()
        print("[Detect] run_detect time=", t1 - t0)
        return preds
    '''
    parse_all: parse all object's detection result
    '''
    def parse_all(self, data):
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
    draw_axis: draw orientation vectors
    '''
    def draw_axis(self, img, p_, q_, colour, scale):
        p = list(p_)
        q = list(q_)
        ## [visualization1]
        angle = math.atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
        hypotenuse = math.sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))

        # Here we lengthen the arrow by a factor of scale
        q[0] = p[0] - scale * hypotenuse * math.cos(angle)
        q[1] = p[1] - scale * hypotenuse * math.sin(angle)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)

        # create the arrow hooks
        p[0] = q[0] + 9 * math.cos(angle + math.pi / 4)
        p[1] = q[1] + 9 * math.sin(angle + math.pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)

        p[0] = q[0] + 9 * math.cos(angle - math.pi / 4)
        p[1] = q[1] + 9 * math.sin(angle - math.pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
        ## [visualization1]
    '''
    get_orientation: get object's orientation from countour
    '''
    def get_orientation(self, pts, img):
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

        # Store the center of the object
        cntr = (int(mean[0,0]), int(mean[0,1]))
        ## [pca]

        ## [visualization]
        # Draw the principal components
        cv2.circle(img, cntr, 3, (255, 0, 255), 2)
        p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
        p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
        self.draw_axis(img, cntr, p1, (0, 255, 0), 1)
        self.draw_axis(img, cntr, p2, (255, 255, 0), 5)

        angle = math.atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
        ## [visualization]

        return angle
    '''
    process_angle: get angle and draw contour, orientation from object image
    '''
    def process_angle(self, src):
        # Convert image to grayscale
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

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

        # Draw each contour only for visualisation purposes
        cv2.drawContours(src, contours, 0, (0, 0, 255), 2)
        # Find the orientation of each shape
        angle = self.get_orientation(l_c, src)

        return (src, angle)
    '''
    detect: general detect work, including capture frame, run detect, draw boxes, get orientation,...
    '''
    def detect(self):
        # Start counting time
        t0 = time.time()

        # Capture frame
        ret, inputImage = self.cap.read()
        if not ret:
            print("[Detect] Frame capture error!")
            return False

        # Get the middle 704x704 part of the image
        inputImage = inputImage[0:704, 288:992]

        # Run detection
        outs = self.run_detect(inputImage)

        # Parse result
        class_ids, confidences, boxes = self.parse_all(outs[0])

        # Return if no object detected
        if len(class_ids) < 1:
            return False

        # Get the object closest to the right on the image
        target_obj_id = 0
        target_obj_x0 = 0
        for i in range(len(boxes)):
            x0 = boxes[i][1].item()
            if x0 > target_obj_x0:
                # If current object is closer to the right than the previous one
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

        x0 = box[0].item()
        w  = box[2].item()
        x1 = x0 + w

        # Print information about the selected object
        print(f"[Object] Object selected: {self.class_names[class_id]} @ {x0},{y0},{x1},{y1} conf={confidence}")

        # Calculate object's orientation angle
        obj = inputImage[y0:y1, x0:x1] # Get only the object's image
        outimg, o_angle = self.process_angle(obj)
        inputImage[y0:y1, x0:x1] = outimg # Replace the object's image with the one has orientation drawn
        o_angle = math.degrees(o_angle) # Convert to degree

        print(f"[Object] Object orientation angle: {o_angle} deg")

        # Draw selected object's bounding box
        color = self.colors[int(class_id) % len(self.colors)]
        cv2.rectangle(inputImage, box, color, 2)
        cv2.rectangle(inputImage, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
        cv2.putText(inputImage, self.class_names[class_id], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))

        # Show the image
        cv2.imshow("output", inputImage)

        # Calculate base's angle to move to (from home location)
        # First, calculate center point of the object (in pixel)
        c_y = y0 + h//2
        c_x = x0 + w//2

        # Calculate distance (Oy) from object's center to arm base
        # Oy distance from image's top edge to arm base is specified in TE2AB_Y (in cm)
        d_y = c_y*P2C + TE2AB_Y

        # Calculate distance (Ox) from object's center to arm base
        # arm base's x is specified in AB_X (in pixel)
        d_x = abs(c_x - AB_X)*P2C

        # Calculate angle from the center, convert it to degree and direction to move
        base_a = 90 - math.degrees(math.atan(d_y/d_x))

        # If move to the right, then set the angle to negative
        # (just my stupid convention, not a math stuff)
        if (c_x - OC2AB_X) < 0:
            base_a = -base_a

        print(f"[Object] Target base angle: {base_a}")

        # Estimate the distance from the object to the arm's base
        distance = math.sqrt(d_x**2 + d_y**2)
        print(f"[Object] Estimated distance: {distance}")

        # Print the time needed for the whole calculation process
        t1 = time.time()
        print("[Detect] detect time=", t1 - t0)

        return base_a, distance, o_angle 
