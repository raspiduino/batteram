# This file contains real life constants
# Created by gvl610

# Real life constants
P2C = 1/37 # cm = pixel * p2c
AB_X = 513 # Arm base's x (in pixel)
DEG2PWM = 2 # degree to PWM. 1 degree = 2 pwm value
D2Y = {16: 525, 27: 106} # Map distance to y (in pixel) on the image
BELT_SPEED = 100 # Conveyor belt speed (pixels/sec)

# Input image constants
CAM_ID = 1
CAM_W = 1280
CAM_H = 720

# Image processing contants
INPUT_WIDTH = 704
INPUT_HEIGHT = 704
MODEL_NAME = "models\\v3.onnx" # Choose your model weight
SCORE_THRESHOLD = 0.9
NMS_THRESHOLD = 0.8
CLASS_NAMES = ["9V", "AA"]
BB_COLORS = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]

# Proximity sensor's x location range on the image (in pixel)
PROX_START = 519
PROX_END = 704

# Default init PWM values
INIT_PWM = [115,354,72,170,500,0,0]
GRIP_ON = 70
GRIP_OFF = 500

# servo 1 pwm values for each actions
DROP_S1_PWM = 115
PICK_S1_PWM = 300

# servo 2 & 3 pwm values for each actions
PREPARE_PWM = (354, 72)
DROP_PWM = (285, 72)
PWM_TABLE = {16: (308, 100), 17: (288, 97), 18: (277, 97), 19: (268, 95), 20: (258, 88), 21: (245, 83), 22: (241, 78), 23: (235, 76), 24: (226, 70)}

# Conveyor belt values
BELT_FORWARD = (0, 4095)
BELT_BACKWARD = (4095, 0)
BELT_STOP = (0, 0)

# Arm limit range
ARM_LIMITX1 = 0
ARM_LIMITY1 = 100
ARM_LIMITX2 = 700
ARM_LIMITY2 = 600

# Arm physical length
ARM_M = 13 # servo2's joint's length
ARM_N = 14 # servo3's joint's length
ARM_H = 4  # delta height between gripper's root and arm's root
