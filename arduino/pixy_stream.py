import serial

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

print("Listening for Pixy2 data...\n")

try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            try:
                x, y = map(int, line.split(','))
                print(f"X: {x}  Y: {y}")
            except ValueError:
                print("Malformed line:", line)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
