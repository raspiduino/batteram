# This file contains code for communicating (via websocket) with esp32, for controlling arm and conveyor belt.
# Created by gvl610

from websocket import WebSocket
from time import sleep
import math
from const import *

class Arm():
    def __init__(self):
        self.ws = WebSocket()
        self.pwm = INIT_PWM
    def reconnect(self):
        try:
            self.ws.connect(self.address)
            print("[Control] Connected to esp32 websocket @ " + self.address)
            
            # Sending default values
            self.send()
            return True
        except ConnectionRefusedError():
            # Wait 3 seconds, then connect again
            print("[Control] Failed to connect to esp32 websocket @ " + self.address + ", reconnecting in 3 seconds...")
            sleep(3)
            self.reconnect()
    def connect(self, address: str):
        self.addr = address
        self.reconnect()
    def send(self):
        # Prepare payload
        payload = ",".join([str(i) for i in self.pwm])
        print("[Control] PWM: " + payload)

        # Send payload
        if self.ws.connected != True:
            # Reconnect if disconnected
            self.reconnect()
        self.ws.send(payload)

        if self.ws.recv() == "success":
            return True
    # Helper function for calculating servo2 and servo3 angle based on the distance
    def dist2angle(self, d):
        # servo2's angle and servo3's angle are measured as drawn in this link:
        # https://www.geogebra.org/classic/dmfz2fww
        c = math.acos((ARM_M**2 + ARM_N**2 - ARM_H**2 - d**2)/(2*ARM_M*ARM_N))
        t = math.sin(c)*ARM_N
        a = 2*math.atan2((math.sqrt(-t**2 + d**2 + ARM_H**2) - ARM_H)/(t+d))
        b = c - a
        return (math.degrees(a), math.degrees(b))
    '''
    Move to position
    basea: arm base's angle (in degree)
    distance: distance from arm's root to object
    obja: gripper's angle (in degree)
    gripp: gripper's PWM value for gripping object
    '''
    def moveto(self, base_a, distance, o_angle, gripp):
        # Process base angle
        # If basea >= 0 -> move to the left
        # Else move to the right
        # (just my stupid convention, not a math stuff)
        if base_a >= 0:
            self.pwm[0] = INIT_PWM[0] - round(base_a * DEG2PWM)
        else:
            self.pwm[0] = round(base_a * DEG2PWM) - INIT_PWM[0]
        
        # Calculate arm angle to reach the desired distance
        s2a, s3a = self.dist2angle(distance)
        # TODO: get the root pwm and root angle and calculate
        
        # Calculate the gripper angle based on the object angle
        # obja has weird format: it increases clockwise, and takes Ox as divider between positive and negative.
        # Basically it's reverse of normal trigeometry circle
        
        # Grip
        self.pwm[4] = gripp

        # Send
        self.send()
