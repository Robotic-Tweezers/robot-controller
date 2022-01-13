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

def load_window(port_name, steppers):
    layout = [
        [Gui.Text("Robot Tweezers Test GUI")],
        [Gui.Text("Connected to: " + port_name), Gui.Button("Reconnect")],
        [Gui.Text("Stepper Settings"), Gui.InputText(), Gui.Button("Download Settings")]
    ]
    for i in range(steppers):
        layout.append([Gui.Text("Stepper %d" % i), Gui.InputText(default_text=0, size=(15, 30)), Gui.Button("Goto")])

    return Gui.Window(title="Robot Tweezers Test GUI", layout=layout, margins=(400, 300))

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
    port.baudrate = 9600
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
        write_json(port, {"name": ""})
        connection_string = read_json(port)
        return connection_string, port

def main():
    firmware, tweezers_port = connect()
    print("Connected to " + firmware['name'] + " on port " + tweezers_port.name)
    layout = [[Gui.Text(firmware['name'], size=(10, 10))], [Gui.Button("Toggle LED")]]
    window = load_window(tweezers_port.name, 3)
    led = True
    while True:
        event, values = window.read()
        # End program if user closes window or
        # presses the OK button
        if event == "Toggle LED":
            if led:
                led = False
            else:
                led = True
            write_json(tweezers_port, {"led_state": led})
        if event == Gui.WIN_CLOSED:
            break
        
    window.close()

if __name__ == '__main__':
    main()