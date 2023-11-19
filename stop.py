from websocket import WebSocket
from const import INIT_PWM
import os

ws = WebSocket()
ws.connect("ws://192.168.43.199/ws")
ws.send(",".join([str(i) for i in INIT_PWM]))
ws.close()
