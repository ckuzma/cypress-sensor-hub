import os
import serial

with serial.Serial('COM3', timeout=3.0) as ser:
    while True:
        data = ser.readline().decode('utf-8')
        data = data.split(',')
        os.system('cls')
        known_readings = {
            "temp": data[6]
        }
        for point in data:
            print(point)
