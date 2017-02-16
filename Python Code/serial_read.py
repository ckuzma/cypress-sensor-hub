import json
import os
import serial

raw_print = False
pretty_print = True

with serial.Serial('COM3', timeout=3.0) as ser:
    while True:
        data = ser.readline().decode('utf-8')
        data = data.split(',')
        os.system('cls')
        known_readings = {
            "temp": data[6],
            "amb_light": data[8]
        }

        if pretty_print is True:
            print(json.dumps(known_readings, indent=2))

        if raw_print is True:
            for point in data:
                print(point)
