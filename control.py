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
        self.address = address
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
