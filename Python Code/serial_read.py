import datetime
import os
import serial
import sys
import time

raw_print = True

def value(data, index):
    return int(data[index + 1]) * 255 + int(data[index])

def loop(port):
    timestamp = '{:%Y-%m-%d}'.format(datetime.datetime.now())
    raw = open(timestamp + '.csv', 'w')
    raw.write('time,temp,ambient light,conduct\n')
    with serial.Serial(port, timeout=3.0) as ser:
        while True:
            data = ser.readline().decode('utf-8')
            data = data.split(',')
            try:
                timestamp = '{:%Y-%m-%d %H:%M:%S}'.format(datetime.datetime.now())
                temp = value(data, 6)
                amb_light = value(data, 8)
                conduct = value(data, 24)
                csv_line = timestamp + ',' + str(temp) + ',' + str(amb_light) + ',' + str(conduct) + '\n'
                raw.write(csv_line)
            except:
                print('Error: Unable to receive data')

            if raw_print is True:
                if os.name == 'nt':
                    os.system('cls')
                else:
                    os.system('clear')
                print('temp:')
                print('\t' + str(temp))
                print('amb_light:')
                print('\t' + str(amb_light))
                print('conduct:')
                print('\t' + str(conduct))
                x = 0
                while x < len(data) - 1:
                    if x != 6 and x != 8 and x != 24:
                        print(str(x) + ' - ' + str(value(data, x)))
                    x+=2

if __name__ == '__main__':
    def usage():
        print('\n\tUsage:')
        print('\tpython ' + sys.argv[0] + ' <Arduino_Port>\n')
        exit()
    if len(sys.argv) != 2:
        usage()

    try:
        loop(sys.argv[1])
    except:
        print('\n\tError. Unable to open port.\n')
