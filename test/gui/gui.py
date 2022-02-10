import sys
import os
import serial
import serial.tools.list_ports as SerialPort
import PySimpleGUI as Gui
import json
from time import sleep

from serial.tools.list_ports_windows import NULL

tweezers_port = serial.Serial()
connection_string = ""
ROLL_DEF = 3.14159265359

def load_window(port):
    layout = [
        [Gui.Text("Robot Tweezers Test GUI")],
        [Gui.Text("Connected to: " + port.name), Gui.Button("Reconnect")],
        [Gui.Text("Actuator Settings"), Gui.InputText(key="actuator_settings")],
        [Gui.Text("Roll Angle"), Gui.InputText(key="roll", default_text=3.1415, size=(15, 30))],
        [Gui.Text("Pitch Angle"), Gui.InputText(key="pitch", default_text=0, size=(15, 30))],
        [Gui.Text("Yaw Angle"), Gui.InputText(key="yaw", default_text=0, size=(15, 30))],
        [Gui.Button("Go")]
    ]

    window = Gui.Window(title="Robot Tweezers Test GUI", layout=layout, margins=(400, 300))
    
    while True:
        event, values = window.read()
        # End program if user closes window or
        # presses the OK button
        if event == "Go":
            command = {"position": {}}
            command["position"]["roll"] = values["roll"]
            command["position"]["pitch"] = values["pitch"]
            command["position"]["yaw"] = values["yaw"]
            write_json(port, command)
        if event == Gui.WIN_CLOSED:
            break
        
    window.close()

def read_json(port):
    received = port.inWaiting()
    timeout = 5
    while received == 0 and timeout > 0:
        sleep(0.5)
        timeout -= 1
        received = port.inWaiting()
    raw_data = port.read(received)
    if len(raw_data) == 0:
        return ""
    return json.loads(raw_data)

def write_json(port, data):
    string_data = json.JSONEncoder().encode(data)
    byte_array = bytes(string_data, "utf-8")
    port.write(byte_array)
    
def serial_init(name):
    port = serial.Serial(name, timeout=1)
    port.baudrate = 921600
    port.bytesize = 8
    port.parity = serial.serialutil.PARITY_NONE
    port.stopbits = 1
    return port

def connect():
    ports = SerialPort.comports()
    while not ports:
        print("No serial ports found, retrying...")
        sleep(3)
        ports = SerialPort.comports()

    for port in ports:
        port = serial_init(port.name)
        write_json(port, {"version": ""})
        connection_string = read_json(port)
        if connection_string == "":
            continue
        if connection_string["version"].startswith("Robot Tweezers"):
            return connection_string, port

    return NULL, NULL

def main():
    firmware, tweezers_port = connect()
    if firmware is NULL:
        return
    print("Connected to " + firmware['version'] + " on port " + tweezers_port.name)
    load_window(tweezers_port)

if __name__ == '__main__':
    main()