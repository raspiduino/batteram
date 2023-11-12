import tkinter as tk
from websocket import WebSocket
from time import sleep

class Arm():
    def __init__(self):
        self.ws = WebSocket()
        self.pwm = [300,354,72,170,500,0,0]
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

arm = Arm()

def set0(value):
    print(f"Slider 0 value: {value}")
    arm.pwm[0] = value
    arm.send()

def set1(value):
    print(f"Slider 1 value: {value}")
    arm.pwm[1] = value
    arm.send()

def set2(value):
    print(f"Slider 2 value: {value}")
    arm.pwm[2] = value
    arm.send()

def set3(value):
    print(f"Slider 3 value: {value}")
    arm.pwm[3] = value
    arm.send()

def set4(value):
    print(f"Slider 4 value: {value}")
    arm.pwm[4] = value
    arm.send()

def main():
    root = tk.Tk()
    root.title("Slider")
    root.geometry("640x480")

    arm.connect("ws://192.168.43.199/ws")

    # Slider 0
    slider1 = tk.Scale(root, from_=70, to=500, orient=tk.HORIZONTAL, command=set0, length=430)
    slider1.set(arm.pwm[0])
    slider1.pack(pady=10)

    # Slider 1
    slider1 = tk.Scale(root, from_=70, to=500, orient=tk.HORIZONTAL, command=set1, length=430)
    slider1.set(arm.pwm[1])
    slider1.pack(pady=10)

    # Slider 2
    slider2 = tk.Scale(root, from_=70, to=500, orient=tk.HORIZONTAL, command=set2, length=430)
    slider2.set(arm.pwm[2])
    slider2.pack(pady=10)

    # Slider 3
    slider3 = tk.Scale(root, from_=70, to=500, orient=tk.HORIZONTAL, command=set3, length=430)
    slider3.set(arm.pwm[3])
    slider3.pack(pady=10)

    # Slider 4
    slider4 = tk.Scale(root, from_=70, to=500, orient=tk.HORIZONTAL, command=set4, length=430)
    slider4.set(arm.pwm[4])
    slider4.pack(pady=10)

    root.mainloop()

if __name__ == "__main__":
    main()
